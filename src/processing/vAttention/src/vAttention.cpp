/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Chiara Bartolozzi
 * email:  chiara.bartolozzi@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "vAttention.h"
#include <iomanip>
#include <fstream>

using namespace yarp::math;
using namespace std;

bool vAttentionModule::configure(yarp::os::ResourceFinder &rf) {
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vAttention")).asString();
    yarp::os::RFModule::setName(moduleName.c_str());

    bool strictness = rf.check("strictness") &&
                      rf.check("strictness", yarp::os::Value(true)).asBool();

    /* attach a port of the same name as the module (prefixed with a /) to the module
     so that messages received from the port are redirected to the respond method */

    std::string handlerPortName = "/";
    handlerPortName += getName();         // use getName() rather than a literal

    if (!handlerPort.open(handlerPortName.c_str())) {
        std::cout << getName() << ": Unable to open port " << handlerPortName << std::endl;
        return false;
    }

    attach(handlerPort);                  // attach to port

    /* set parameters */
    int sensorWidth = rf.check("sensorWidth", yarp::os::Value(304)).asInt();
    int sensorHeight = rf.check("sensorHeight", yarp::os::Value(240)).asInt();
    double tau = rf.check("tau", yarp::os::Value(8.0)).asDouble();
    double thrSal = rf.check("thr", yarp::os::Value(20)).asDouble();
    string filtersPath = rf.check("filtersPath", yarp::os::Value("../../src/processing/vAttention/filters/")).asString();

    /* create the thread and pass pointers to the module parameters */
    attManager = new vAttentionManager(sensorWidth, sensorHeight, tau, thrSal, filtersPath);

    return attManager->open(moduleName, strictness);

}

bool vAttentionModule::interruptModule() {
    attManager->interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

bool vAttentionModule::close() {
    attManager->close();
    delete attManager;
    yarp::os::RFModule::close();
    return true;
}

bool vAttentionModule::updateModule() {
    return true;
}

double vAttentionModule::getPeriod() {
    return 1;
}

bool vAttentionModule::respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply) {
    std::string helpMessage = std::string(getName().c_str()) +
                              " commands are: \n" +
                              "help \n" +
                              "quit \n";
    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");
        return false;
    } else if (command.get(0).asString() == "help") {
        std::cout << helpMessage;
        reply.addString("ok");
    }

    return true;
}

/******************************************************************************/
//vAttentionManager
/******************************************************************************/

void vAttentionManager::load_filter(const string filename, yarp::sig::Matrix &filterMap, int &filterSize) {

    //Opening filter file.
    ifstream file;
    file.open(filename.c_str(), ios::in | ios::out);
    if (!file.is_open()) {
        std::cerr << "Could not open filter file " << filename << std::endl;
    }

    string line;
    int r = 0;
    int c = 0;

    //File is parsed line by line. Values are separated by spaces
    while (!std::getline(file, line, '\n').eof()) {
        istringstream reader(line);
        string::const_iterator i = line.begin();
        if (line.empty())
            continue;
        c = 0;

        while (!reader.eof()) {

            double val;
            reader >> val;

            //Resize the map to contain new values if necessary
            if (r + 1 > filterMap.rows()) {
                filterMap.resize(r + 1, filterMap.cols());
            }
            if (c + 1 > filterMap.cols()) {
                filterMap.resize(filterMap.rows(), c + 1);
            }

            filterMap(r, c) = val;
            c++;
        }
        r++;
    }

    //The returned filterSize is updated with the maximum dimension of the filter
    int maxDimension = std::max(filterMap.rows(), filterMap.cols());
    filterSize = max(filterSize, maxDimension);
}

vAttentionManager::vAttentionManager(int sensorWidth, int sensorHeight, double tau, double thrSal,
                                     std::string &filtersPath) {
    this->sensorWidth = sensorWidth;
    this->sensorHeight = sensorHeight;
    this->tau = tau;
    this->thrSal = thrSal;
    this->prevT = yarp::os::Time::now();
    normSal = thrSal / 255;

    filterSize = 0;

    int gaborSize = 15;
    double sigmaGabor =(double) gaborSize / 4.0;
    double fGabor = 2.0 /(double) gaborSize;
    double amplGabor = 0.4;
//    amplGabor *= 100;

    filterSize = max(gaborSize, filterSize);
    //Generating Gabor filters
    generateGaborFilter(gabor0, gaborSize, amplGabor, fGabor, sigmaGabor, 0);
    generateGaborFilter(gabor45, gaborSize, amplGabor, fGabor, sigmaGabor, 45);
    generateGaborFilter(gabor90, gaborSize, amplGabor, fGabor, sigmaGabor, 90);
    generateGaborFilter(gabor135, gaborSize, amplGabor, fGabor, sigmaGabor, 135);

    gaborSize = 11;
    fGabor = 1.0 /(double) gaborSize;
    sigmaGabor =(double) gaborSize / 4.0;
    amplGabor = -1.5;
    filterSize = max(gaborSize, filterSize);

//    amplGabor *= 100;
    generateGaborFilter(negGabor0, gaborSize, amplGabor, fGabor, sigmaGabor, 0);
    generateGaborFilter(negGabor45, gaborSize, amplGabor, fGabor, sigmaGabor, 45);
    generateGaborFilter(negGabor90, gaborSize, amplGabor, fGabor, sigmaGabor, 90);
    generateGaborFilter(negGabor135, gaborSize, amplGabor, fGabor, sigmaGabor, 135);


    this->salMapPadding = filterSize / 2 + filterSize %2;

    //for speed we predefine the memory for some matrices
    //The saliency map is bigger than the image of the maximum size among the loaded filters
    boxWidth = 2;
    boxHeight = 2;

    int mapWidth = sensorWidth / boxWidth + 2 * salMapPadding;
    int mapHeight = sensorHeight / boxHeight + 2 * salMapPadding;

    eventMap = yarp::sig::Matrix(sensorHeight / boxHeight, sensorWidth / boxWidth);
    salMapLeftEdges = yarp::sig::Matrix(mapHeight, mapWidth);
    salMapLeftSpread = yarp::sig::Matrix(mapHeight, mapWidth);
    salMapRight = yarp::sig::Matrix(mapHeight, mapWidth);
    gabor0FeatureMap = yarp::sig::Matrix(mapHeight, mapWidth);
    gabor45FeatureMap = yarp::sig::Matrix(mapHeight, mapWidth);
    gabor90FeatureMap = yarp::sig::Matrix(mapHeight, mapWidth);
    gabor135FeatureMap = yarp::sig::Matrix(mapHeight, mapWidth);
    negGabor0FeatureMap = yarp::sig::Matrix(mapHeight, mapWidth);
    negGabor45FeatureMap = yarp::sig::Matrix(mapHeight, mapWidth);
    negGabor90FeatureMap = yarp::sig::Matrix(mapHeight, mapWidth);
    negGabor135FeatureMap = yarp::sig::Matrix(mapHeight, mapWidth);
    activationMap = yarp::sig::Matrix(mapHeight, mapWidth);

    salMapLeftEdges.zero();
    salMapRight.zero();
    activationMap.zero();
}


void vAttentionManager::generateOrientedGaussianFilter(yarp::sig::Matrix &filterMap, double A, double sigmaX, double sigmaY,
                                                       double theta, int gaussianFilterSize, int xCenter, int yCenter) {
    //Resize to desired size
    filterMap.resize(gaussianFilterSize, gaussianFilterSize);
    filterMap.zero();

    double theta_rad = theta * M_PI /180;
    double a = cos(theta_rad);
    double b = sin(theta_rad);
    double c = cos (theta_rad + M_PI_2);
    double d = sin (theta_rad + M_PI_2);

    //Generate gaussian filter
    for (int row = 0; row < gaussianFilterSize; row++) {
        for (int col = 0; col < gaussianFilterSize; col++) {

            double rDist = row - yCenter;
            double cDist = col - xCenter;
            filterMap(row, col) =A * exp(-(pow((a*rDist+b*cDist), 2) / (2 * pow(sigmaX, 2))
                                           + pow((c*rDist+d*cDist), 2) / (2 * pow(sigmaY, 2))));
        }
    }
}

void vAttentionManager::generateGaborFilter(yarp::sig::Matrix &filterMap, int gaborFilterSize, double A, double f,
                                            double sigma, double theta)
{
    filterMap.resize(gaborFilterSize, gaborFilterSize);
    filterMap.zero();

    double th_r = theta *M_PI/180.0;

    for (double x = 0; x < gaborFilterSize; ++x) {
        for (double y = 0; y < gaborFilterSize; ++y) {

            double  x1 = x - (double)gaborFilterSize/2.0;
            double  y1 = y - (double)gaborFilterSize/2.0;

            filterMap(x,y) =A * exp(-(pow(x1,2.0)+pow(y1,2.0))/(2*pow(sigma,2))) * cos(2.0*M_PI*f*(x1*cos(th_r)+y1*sin(th_r)));
        }
    }
}
bool vAttentionManager::open(const std::string moduleName, bool strictness) {
    this->strictness = strictness;
    if (strictness) {
        std::cout << "Setting " << moduleName << " to strict" << std::endl;
        this->setStrict();
    }
    this->useCallback();

    // why is the input port treated differently???? both in open and close
    std::string inPortName = "/" + moduleName + "/vBottle:i";
    bool check = BufferedPort<ev::vBottle>::open(inPortName);

    if (strictness) outPort.setStrict();
    std::string outPortName = "/" + moduleName + "/vBottle:o";
    check &= outPort.open(outPortName);

    outPortName = "/" + moduleName + "/salMapLeft:o";
    check &= outSalMapLeftPort.open(outPortName);

    outPortName = "/" + moduleName + "/activationMap:o";
    check &= outActivationMapPort.open(outPortName);

    outPortName = "/" + moduleName + "/gabor0:o";
    check &= outGabor0Port.open(outPortName);

    outPortName = "/" + moduleName + "/gabor45:o";
    check &= outGabor45Port.open(outPortName);

    outPortName = "/" + moduleName + "/gabor90:o";
    check &= outGabor90Port.open(outPortName);

    outPortName = "/" + moduleName + "/gabor135:o";
    check &= outGabor135Port.open(outPortName);

    outPortName = "/" + moduleName + "/negGabor0:o";
    check &= outNegGabor0Port.open(outPortName);

    outPortName = "/" + moduleName + "/negGabor45:o";
    check &= outNegGabor45Port.open(outPortName);

    outPortName = "/" + moduleName + "/negGabor90:o";
    check &= outNegGabor90Port.open(outPortName);

    outPortName = "/" + moduleName + "/negGabor135:o";
    check &= outNegGabor135Port.open(outPortName);

    std::cout << "\ninitialisation correctly ended" << std::endl;

    return check;
}

void vAttentionManager::close() {
    //close ports
    outPort.close();
    yarp::os::BufferedPort<ev::vBottle>::close();
    outSalMapLeftPort.close();
    outActivationMapPort.close();
}

void vAttentionManager::interrupt() {
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    yarp::os::BufferedPort<ev::vBottle>::interrupt();
    outSalMapLeftPort.interrupt();
    outActivationMapPort.interrupt();
}

void vAttentionManager::onRead(ev::vBottle &bot) {
//    numIterations ++;
    /* get the event queue in the vBottle bot */
    ev::vQueue q = bot.get<ev::AddressEvent>();

    q.sort(true);

    int x,y, xScaled, yScaled;
    eventMap.zero();
    for (ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {
        ev::event<ev::AddressEvent> aep = ev::getas<ev::AddressEvent>(*qi);

        if (!aep) continue;
        //TODO handle left and right salMap

        x = aep->getX();
        y = aep->getY();
        if (aep->getChannel() == 0) {
            xScaled = x / boxWidth;
            yScaled = y / boxHeight;
            eventMap(yScaled, xScaled) ++;
        } else {
            //TODO
        }
    }

    int IORSize = 5;
    double ampl;
    int topRowIOR = attPointRow - IORSize;
    int bottomRowIOR = attPointRow + IORSize;
    int topColIOR = attPointCol - IORSize;
    int bottomColIOR = attPointCol + IORSize;

    for (int r = 0; r < eventMap.rows(); ++r) {
        for (int c = 0; c < eventMap.cols(); ++c) {
            ampl = (double) eventMap(r, c);
            if (ampl != 0) {
                updateMap(gabor0FeatureMap, ampl * gabor0, r, c, topRowIOR, topColIOR, bottomRowIOR, bottomColIOR);
                updateMap(gabor45FeatureMap, ampl * gabor45, r, c, topRowIOR, topColIOR, bottomRowIOR, bottomColIOR);
                updateMap(gabor90FeatureMap, ampl * gabor90, r, c, topRowIOR, topColIOR, bottomRowIOR, bottomColIOR);
                updateMap(gabor135FeatureMap, ampl * gabor135, r, c, topRowIOR, topColIOR, bottomRowIOR, bottomColIOR);
//                updateMap(negGabor0FeatureMap, ampl * negGabor0, r, c, topRowIOR, topColIOR, bottomRowIOR, bottomColIOR);
//                updateMap(negGabor45FeatureMap, ampl * negGabor45, r, c, topRowIOR, topColIOR, bottomRowIOR, bottomColIOR);
//                updateMap(negGabor90FeatureMap, ampl * negGabor90, r, c, topRowIOR, topColIOR, bottomRowIOR, bottomColIOR);
//                updateMap(negGabor135FeatureMap, ampl * negGabor135, r, c, topRowIOR, topColIOR, bottomRowIOR, bottomColIOR);
            }
        }
    }

    unsigned long int t = unwrap (q.back().get()->getStamp());

    double dt = (double)(t - prevT) * (80*pow(10,-9));
    prevT = t;

    decayMap(gabor0FeatureMap, dt);
    decayMap(gabor45FeatureMap, dt);
    decayMap(gabor90FeatureMap, dt);
    decayMap(gabor135FeatureMap, dt);

    decayMap(negGabor0FeatureMap, dt);
    decayMap(negGabor45FeatureMap, dt);
    decayMap(negGabor90FeatureMap, dt);
    decayMap(negGabor135FeatureMap, dt);

    decayMap(activationMap, dt);

//    threshold(gabor0FeatureMap, gabor0FeatureMap, 0);
//    threshold(gabor45FeatureMap, gabor45FeatureMap, 0);
//    threshold(gabor90FeatureMap, gabor90FeatureMap, 0);
//    threshold(gabor135FeatureMap, gabor135FeatureMap, 0);
//
//    threshold(negGabor0FeatureMap, negGabor0FeatureMap, 0);
//    threshold(negGabor45FeatureMap, negGabor45FeatureMap, 0);
//    threshold(negGabor90FeatureMap, negGabor90FeatureMap, 0);
//    threshold(negGabor135FeatureMap, negGabor135FeatureMap, 0);

    double th = 50;
    bool binary = false;
    threshold(gabor0FeatureMap, threshGabor0FeatureMap, th, binary);
    threshold(gabor45FeatureMap, threshGabor45FeatureMap, th, binary);
    threshold(gabor90FeatureMap, threshGabor90FeatureMap, th, binary);
    threshold(gabor135FeatureMap, threshGabor135FeatureMap, th, binary);

    threshold(negGabor0FeatureMap, threshNegGabor0FeatureMap, th, binary);
    threshold(negGabor45FeatureMap, threshNegGabor45FeatureMap, th, binary);
    threshold(negGabor90FeatureMap, threshNegGabor90FeatureMap, th, binary);
    threshold(negGabor135FeatureMap, threshNegGabor135FeatureMap, th, binary);

    if (binary) {
        threshGabor0FeatureMap *= 255;
        threshGabor45FeatureMap *= 255;
        threshGabor90FeatureMap *= 255;
        threshGabor135FeatureMap *= 255;

        threshNegGabor0FeatureMap *= 255;
        threshNegGabor45FeatureMap *= 255;
        threshNegGabor90FeatureMap *= 255;
        threshNegGabor135FeatureMap *= 255;
    }
    //  --- convert to images for display --- //
    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageLeft = outSalMapLeftPort.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageActivation = outActivationMapPort.prepare();

    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageGabor0 = outGabor0Port.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageGabor45 = outGabor45Port.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageGabor90 = outGabor90Port.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageGabor135 = outGabor135Port.prepare();

    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageNegGabor0 = outNegGabor0Port.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageNegGabor45 = outNegGabor45Port.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageNegGabor90 = outNegGabor90Port.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageNegGabor135 = outNegGabor135Port.prepare();



    convertToImage(threshGabor0FeatureMap, imageGabor0, salMapPadding);
    convertToImage(threshGabor45FeatureMap, imageGabor45, salMapPadding);
    convertToImage(threshGabor90FeatureMap, imageGabor90, salMapPadding);
    convertToImage(threshGabor135FeatureMap, imageGabor135, salMapPadding);

    convertToImage(threshNegGabor0FeatureMap, imageNegGabor0, salMapPadding);
    convertToImage(threshNegGabor45FeatureMap, imageNegGabor45, salMapPadding);
    convertToImage(threshNegGabor90FeatureMap, imageNegGabor90, salMapPadding);
    convertToImage(threshNegGabor135FeatureMap, imageNegGabor135, salMapPadding);

    /* DEBUG STUFF
    convertToImage(gabor0, imageGabor0,0);
    convertToImage(gabor45, imageGabor45, 0);
    convertToImage(gabor90, imageGabor90, 0);
    convertToImage(gabor135, imageGabor135, 0);

    convertToImage(negGabor0, imageNegGabor0, 0);
    convertToImage(negGabor45, imageNegGabor45, 0);
    convertToImage(negGabor90, imageNegGabor90, 0);
    convertToImage(negGabor135, imageNegGabor135, 0);



    salMapLeftEdges.zero();
    for (int i = 100; i < 150; ++i) {
        for (int j = 100; j < 150; ++j) {
            salMapLeftEdges(i,j) = 255;
        }
    }
    computeBoundingBox(salMapLeftEdges, 10, 125, 125, topRow, topCol, bottomRow, bottomCol);
    convertToImage(salMapLeftEdges, imageLeft, salMapPadding, 125,125);
*/

    salMapLeftEdges = threshGabor0FeatureMap + threshGabor45FeatureMap + threshGabor90FeatureMap + threshGabor135FeatureMap;
    normaliseMap(salMapLeftEdges,salMapLeftEdges);
    salMapLeftSpread = threshNegGabor0FeatureMap + threshNegGabor45FeatureMap + threshNegGabor90FeatureMap + threshNegGabor135FeatureMap,
    normaliseMap(salMapLeftSpread,salMapLeftSpread);

//            salMapLeftEdges = salMapLeftSpread;
    salMapLeftEdges*= 200000;

//    threshold(salMapLeftEdges, salMapLeftEdges, 50);
    int r1 = salMapPadding;
    int c1 = salMapPadding;
    int r2 = salMapLeftEdges.rows() - salMapPadding;
    int c2 = salMapLeftEdges.cols() - salMapPadding;
    computeAttentionPoint(salMapLeftEdges.submatrix(r1,r2,c1,c2), attPointRow, attPointCol);
    attPointRow += salMapPadding;
    attPointCol += salMapPadding;
//    int rectSize = 15;
//
//    for (int r = attPointRow - rectSize; r < attPointRow + rectSize; ++r) {
//        if (r < 0 || r >= salMapLeftEdges.rows())
//            continue;
//        for (int c = attPointCol - rectSize; c < attPointCol + rectSize; ++c) {
//            if (c < 0 || r >= salMapLeftEdges.cols())
//                continue;
//            salMapLeftEdges(r, c) *= 0.3;
//        }
//    }
//    std::cout << "attPointCol = " << attPointCol << std::endl;
//    std::cout << "attPointRow = " << attPointRow << std::endl;
    int topCol,topRow,bottomCol,bottomRow;
//    computeBoundingBox(salMapLeftEdges, 10, attPointRow, attPointCol, topRow, topCol, bottomRow, bottomCol);
    topRow = attPointRow - IORSize;
    topCol = attPointCol - IORSize;
    bottomRow = attPointRow + IORSize;
    bottomCol = attPointCol + IORSize;
    convertToImage(salMapLeftEdges, imageLeft, salMapPadding, attPointRow, attPointCol);
    drawBoundingBox(imageLeft, topRow - salMapPadding, topCol - salMapPadding, bottomRow - salMapPadding, bottomCol - salMapPadding);

    convertToImage(activationMap, imageActivation, salMapPadding, attPointRow, attPointCol);

    // --- writing images of left and right saliency maps on output port

    if (outSalMapLeftPort.getOutputCount()) {
        outSalMapLeftPort.write();
    }

    if (outActivationMapPort.getOutputCount()) {
        outActivationMapPort.write();
    }

    if (outGabor0Port.getOutputCount()) {
        outGabor0Port.write();
    }
    if (outGabor45Port.getOutputCount()) {
        outGabor45Port.write();
    }
    if (outGabor90Port.getOutputCount()) {
        outGabor90Port.write();
    }
    if (outGabor135Port.getOutputCount()) {
        outGabor135Port.write();
    }
    if (outNegGabor0Port.getOutputCount()) {
        outNegGabor0Port.write();
    }
    if (outNegGabor45Port.getOutputCount()) {
        outNegGabor45Port.write();
    }
    if (outNegGabor90Port.getOutputCount()) {
        outNegGabor90Port.write();
    }
    if (outNegGabor135Port.getOutputCount()) {
        outNegGabor135Port.write();
    }

}

void vAttentionManager::convertToImage(const yarp::sig::Matrix &map, yarp::sig::ImageOf<yarp::sig::PixelBgr> &image,
                                       int mapPaddingSize, int rMax, int cMax) {

    /*prepare output vBottle with images */
    int imageRows = map.rows() - 2 * mapPaddingSize;
    int imageCols = map.cols() - 2 * mapPaddingSize;
    image.resize(imageCols, imageRows);
    image.setTopIsLowIndex(true);
    image.zero();

    int shiftedR, shiftedC;
    for (int r = 0; r < imageRows; r++) {
        for (int c = 0; c < imageCols; c++) {
            yarp::sig::PixelBgr pixelBgr;

//          Coordinates of saliency map are shifted by mapPaddingSize wrt the image
            shiftedR = r + mapPaddingSize;
            shiftedC = c + mapPaddingSize;

            double pixelValue = map(shiftedR, shiftedC);
            //Attention point is highlighted in red, negative values in blue, positive in green
            if (shiftedR == rMax && shiftedC == cMax) {
                pixelBgr.r = 255;
            } else if (pixelValue <= 0) {
                pixelBgr.b = std::min(fabs(pixelValue), 255.0);
            } else {
                pixelBgr.g = std::min(fabs(pixelValue), 255.0);
            }

            image(c, r) = pixelBgr;
        }
    }
}

void vAttentionManager::drawSquare( yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, int r, int c,
                                    yarp::sig::PixelBgr &pixelBgr)  {
    int squareSize = 2;
    for (int i = -squareSize; i <= squareSize; ++i) {
        for (int j = -squareSize; j <= squareSize; ++j) {
            if ((r + i< (image.height() -1)) && r + i>= 0)
                if ((c +j< (image.width() -1)) && c + j>= 0)
                    image(r +i,c+j) = pixelBgr;
        }
    }
}

void vAttentionManager::updateMap(yarp::sig::Matrix &map, const yarp::sig::Matrix &filterMap, int row, int col, int topRow,
                                  int topCol, int bottomRow, int bottomCol) {

    int filterRows = filterMap.rows();
    int filterCols = filterMap.cols();
    int rMap, cMap;
    bool inIOR;
    // ---- increase energy in the location of the event ---- //
    for (int rFil = 0; rFil < filterRows; rFil++) {
        for (int cFil = 0; cFil < filterCols; cFil++) {
            rMap = row + rFil;
            cMap = col + cFil;
            inIOR = (rMap >= topRow && rMap < bottomRow);
            inIOR &= (cMap >= topCol && cMap < bottomCol);
            if (inIOR) {
                map(rMap, cMap) = 0;
            } else {
                map(rMap, cMap) += filterMap(rFil, cFil);
                clamp(map(row + rFil, col + cFil), -2000.0, 2000.0);
            }
        }
    }

}

void vAttentionManager::printMap(const yarp::sig::Matrix &map) {
    for (int r = 0; r < map.rows(); r++) {
        for (int c = 0; c < map.cols(); c++) {
            std::cout << std::setprecision(2) << map(r, c) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void vAttentionManager::decayMap(yarp::sig::Matrix &map, double dt) {
    double decayFactor = exp(- dt / tau);
    map*= decayFactor;
}

void vAttentionManager::normaliseMap(const yarp::sig::Matrix &map, yarp::sig::Matrix &normalisedMap) {
    double totalEnergy;
    normalisedMap = map;
    for (int r = 0; r < map.rows(); ++r) {
        for (int c = 0; c < map.cols(); ++c) {
            totalEnergy += map(r,c);
        }
    }
    normalisedMap /= totalEnergy;
}

void vAttentionManager::computeAttentionPoint(const yarp::sig::Matrix &map, int &attPointRow, int &attPointCol) {

    maxInMap(map, attPointRow, attPointCol);
    activationMap(attPointRow, attPointCol) += 20;
    maxInMap(activationMap, attPointRow, attPointCol);

//    int rectSize = 15;
//
//    for (int r = attPointRow - rectSize; r < attPointRow + rectSize; ++r) {
//        if (r < 0 || r >= activationMap.rows())
//            continue;
//        for (int c = attPointCol - rectSize; c < attPointCol + rectSize; ++c) {
//            if (c < 0 || r >= activationMap.cols())
//                continue;
//            activationMap(r, c) *= 0.5;
//        }
//    }
}

void vAttentionManager::maxInMap(const yarp::sig::Matrix &map, int &rowMax, int &colMax) {
    double max = 0;
    rowMax = 0;
    colMax = 0;
    for (int r = 0; r < map.rows(); r++) {
        for (int c = 0; c < map.cols(); c++) {
            if (map(r, c) > max) {
                max = map(r, c);
                rowMax = r;
                colMax = c;
            }
        }
    }
}


bool vAttentionManager::energyInArea(const yarp::sig::Matrix &map, int topRow, int topCol, int bottomRow, int bottomCol,
                                     double &energy) {
    energy = 0;
    for (int row = topRow; row < bottomRow; ++row) {
        for (int col = topCol; col < bottomCol; ++col) {
            if (row < 0 || col < 0 || row >= map.rows() || col >= map.cols()) {
                energy = 0;
                return false;
            }
            energy += map(row,col);
        }
    }
    return true;
}

void vAttentionManager::computeBoundingBox(const yarp::sig::Matrix &map, double threshold, int centerRow, int centerCol,
                                           int &topRow, int &topCol, int &bottomRow, int &bottomCol) {
    double internalEnergy = 0;
    double previousInternalEnergy = 0;
    double energyTop = 0;
    double energyBottom = 0;
    double energyLeft = 0;
    double energyRight = 0;
    int increase = 5;
    topCol = centerCol - increase;
    topRow = centerRow - increase;
    bottomCol = centerCol + increase;
    bottomRow = centerRow + increase;
    clamp(topCol, 0, map.cols()-1);
    clamp(topRow, 0, map.rows()-1);
    clamp(bottomCol, 0, map.cols()-1);
    clamp(bottomRow, 0, map.rows()-1);

    bool top = energyInArea(map, topRow - increase, topCol, topRow, bottomCol, energyTop);
    bool bottom = energyInArea(map, bottomRow, topCol, bottomRow + increase, bottomCol, energyBottom);
    bool right = energyInArea(map, topRow, bottomCol, bottomRow, bottomCol + increase, energyRight);
    bool left = energyInArea(map, topRow, topCol - increase, bottomRow, topCol, energyLeft);

    energyInArea(map, topRow, topCol, bottomRow, bottomCol, internalEnergy);

    double *maxEnergy;

    while (internalEnergy - previousInternalEnergy > threshold){
        if (!top && !bottom && !left && !right)
            break;

        maxEnergy = &energyTop;
        if (energyBottom > *maxEnergy)
            maxEnergy = &energyBottom;
        if (energyLeft > *maxEnergy)
            maxEnergy = &energyLeft;
        if (energyRight > *maxEnergy)
            maxEnergy = &energyRight;

        if (top && maxEnergy == &energyTop){
            topRow -= increase;
            clamp(topRow, 0, map.rows()-1);
            top = energyInArea(map, topRow - increase, topCol, topRow, bottomCol, energyTop);
        }
        if (bottom && maxEnergy == &energyBottom){
            bottomRow += increase;
            clamp(bottomRow, 0, map.rows()-1);
            bottom = energyInArea(map, bottomRow, topCol, bottomRow + increase, bottomCol, energyBottom);
        }
        if (left && maxEnergy == &energyLeft){
            topCol -= increase;
            clamp(topCol, 0, map.cols()-1);
            left = energyInArea(map, topRow, topCol - increase, bottomRow, topCol, energyLeft);
        }
        if (right && maxEnergy == &energyRight){
            bottomCol += increase;
            clamp(bottomCol, 0, map.cols()-1);
            right = energyInArea(map, topRow, bottomCol, bottomRow, bottomCol + increase, energyRight);
        }
        previousInternalEnergy = internalEnergy;
        energyInArea(map, topRow, topCol, bottomRow, bottomCol, internalEnergy);
    }
    //TODO debug. There is one extra iteration towards the top
}

void vAttentionManager::drawBoundingBox(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, int topRow, int topCol,
                                        int bottomRow, int bottomCol) {
//    bool inRange = true;
//    inRange &= (topRow >= 0 && topRow < image.height());
//    inRange &= (topCol >= 0 && topCol < image.width());
//    inRange &= (bottomRow >= 0 && bottomRow < image.height());
//    inRange &= (bottomCol >= 0 && bottomCol < image.width());
//    if (!inRange)
//        return false;

    if (topRow >= 0 && topRow < image.height())
        for (int i = topCol; i <= bottomCol; ++i) {
            if (i >= 0 && i < image.width())
                image(i, topRow) = yarp::sig::PixelBgr(255,0,0);
        }

    if (bottomRow >= 0 && bottomRow < image.height())
        for (int i = topCol; i <= bottomCol; ++i) {
            if (i >= 0 && i < image.width())
                image(i, bottomRow) = yarp::sig::PixelBgr(255,0,0);
        }

    if (topCol >= 0 && topCol < image.width())
        for (int i = topRow; i <= bottomRow; ++i) {
            if (i >= 0 && i < image.height())
                image(topCol, i) = yarp::sig::PixelBgr(255,0,0);
        }

    if (bottomCol >= 0 && bottomCol < image.width())
        for (int i = topRow; i <= bottomRow; ++i) {
            if (i >= 0 && i < image.height())
                image(bottomCol, i) = yarp::sig::PixelBgr(255,0,0);
        }

//    return true;
};

void vAttentionManager::convolve(const yarp::sig::Matrix &featureMap, yarp::sig::Matrix &convolvedFeatureMap,
                                 const yarp::sig::Matrix &filterMap) {
    int startRow = filterMap.rows()/2 + filterMap.rows() % 2;
    int finalRow = featureMap.rows()- filterMap.rows()/2;
    int startCol = filterMap.cols()/2 + filterMap.cols() % 2;
    int finalCol = featureMap.cols()- filterMap.cols()/2;
    convolvedFeatureMap.zero();
    for (int mapRow = startRow; mapRow < finalRow; ++mapRow) {
        for (int mapCol = startCol; mapCol < finalCol; ++mapCol) {

            for (int filterRow = 0; filterRow < filterMap.rows(); ++filterRow) {
                for (int filterCol = 0; filterCol < filterMap.cols(); ++filterCol) {
                    convolvedFeatureMap(mapRow,mapCol) += featureMap(mapRow + filterRow -startRow, mapCol + filterCol -startCol) * filterMap(filterRow, filterCol);
                }
            }

        }
    }

};

void vAttentionManager::threshold(const yarp::sig::Matrix &map, yarp::sig::Matrix &thresholdedMap, double threshold, bool binary) {
    if (&map != &thresholdedMap){
        thresholdedMap.resize(map.rows(), map.cols());
        thresholdedMap.zero();
    }

    for (int i = 0; i < map.rows(); ++i) {
        for (int j = 0; j < map.cols(); ++j) {
            double &thVal = thresholdedMap(i, j);
            double val = map(i,j);
            if (val < threshold)
                thVal = 0;
            else if (binary){
                thVal = 1;
            } else {
                thVal = map(i,j);
            }
        }
    }
}

template <typename T>
void vAttentionManager::clamp(T &val, T min, T max) {
    val = std::max (min, val);
    val = std::min (max, val);
}


//empty line to make gcc happy
