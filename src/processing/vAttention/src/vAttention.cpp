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
    double tau = rf.check("tau", yarp::os::Value(1.5)).asDouble();
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


//    //Generating gaussian filters
//    double sigmaGauss = 3;
//
//    double gaussAmpl = 10;// 1/(sqrt(2*M_PI) * sigmaGauss);
//    int gaussSize = 24;
//
//    yarp::sig::Matrix gauss1;
//    yarp::sig::Matrix gauss2;
//    yarp::sig::Matrix gauss3;
//
//    generateOrientedGaussianFilter(gauss1, gaussAmpl, sigmaGauss / 4, sigmaGauss, 0, filterSize, gaussSize, gaussSize / 2, gaussSize *2/3);
//    generateOrientedGaussianFilter(gauss2, gaussAmpl, sigmaGauss / 4, sigmaGauss, 0, filterSize, gaussSize, gaussSize / 2, gaussSize / 3);
//    generateOrientedGaussianFilter(gauss3, gaussAmpl, sigmaGauss / 4, sigmaGauss, 0, filterSize, gaussSize, gaussSize/2, gaussSize/2);
//    orient0DOGFilterMap = gauss2 + gauss1 - gauss3;
//
//    generateOrientedGaussianFilter(gauss1, gaussAmpl, sigmaGauss / 4, sigmaGauss, 45, filterSize, gaussSize, gaussSize * 1/3, gaussSize * 1/3);
//    generateOrientedGaussianFilter(gauss2, gaussAmpl, sigmaGauss / 4, sigmaGauss, 45, filterSize, gaussSize, gaussSize * 2/3, gaussSize * 2/3);
//    generateOrientedGaussianFilter(gauss3, gaussAmpl, sigmaGauss / 4, sigmaGauss, 45, filterSize, gaussSize, gaussSize/2, gaussSize/2);
//    orient45DOGFilterMap = gauss2 + gauss1 - gauss3;
//
//    generateOrientedGaussianFilter(gauss1, gaussAmpl, sigmaGauss / 4, sigmaGauss, 90, filterSize, gaussSize, gaussSize * 1/3, gaussSize * 1/2);
//    generateOrientedGaussianFilter(gauss2, gaussAmpl, sigmaGauss / 4, sigmaGauss, 90, filterSize, gaussSize, gaussSize * 2/3, gaussSize * 1/2);
//    generateOrientedGaussianFilter(gauss3, gaussAmpl, sigmaGauss / 4, sigmaGauss, 90, filterSize, gaussSize, gaussSize/2, gaussSize/2);
//    orient90DOGFilterMap = gauss2 + gauss1 - gauss3;
//
//    generateOrientedGaussianFilter(gauss1, gaussAmpl, sigmaGauss / 4, sigmaGauss, 135, filterSize, gaussSize, gaussSize * 2/3, gaussSize * 1/3);
//    generateOrientedGaussianFilter(gauss2, gaussAmpl, sigmaGauss / 4, sigmaGauss, 135, filterSize, gaussSize, gaussSize * 1/3, gaussSize * 2/3);
//    generateOrientedGaussianFilter(gauss3, gaussAmpl, sigmaGauss / 4, sigmaGauss, 135, filterSize, gaussSize, gaussSize/2, gaussSize/2);
//    orient135DOGFilterMap = gauss2 + gauss1 - gauss3;


    int gaborFilterSize = 15;
    double sigmaGabor =(double) gaborFilterSize / 5.5;
    double fGabor = 2.0 /(double) gaborFilterSize;
    double amplGabor = 6;
//    amplGabor *= 100;

    //Generating Gabor filters
    generateGaborFilter(orient0FilterMap, gaborFilterSize, amplGabor, fGabor, sigmaGabor, 0, filterSize);
    generateGaborFilter(orient45FilterMap, gaborFilterSize, amplGabor, fGabor, sigmaGabor, 45, filterSize);
    generateGaborFilter(orient90FilterMap, gaborFilterSize, amplGabor, fGabor, sigmaGabor, 90, filterSize);
    generateGaborFilter(orient135FilterMap, gaborFilterSize, amplGabor, fGabor, sigmaGabor, 135, filterSize);

    gaborFilterSize = 21;
    fGabor = 2.0 /(double) gaborFilterSize;
    sigmaGabor =(double) gaborFilterSize / 5.5;
    amplGabor = -15;

//    amplGabor *= 100;
//    fGabor = 3.0 /(double) filterSize;
    generateGaborFilter(orient0DOGFilterMap, gaborFilterSize, amplGabor, fGabor, sigmaGabor, 0, filterSize);
    generateGaborFilter(orient45DOGFilterMap, gaborFilterSize, amplGabor, fGabor, sigmaGabor, 45, filterSize);
    generateGaborFilter(orient90DOGFilterMap, gaborFilterSize, amplGabor, fGabor, sigmaGabor, 90, filterSize);
    generateGaborFilter(orient135DOGFilterMap, gaborFilterSize, amplGabor, fGabor, sigmaGabor, 135, filterSize);


    this->salMapPadding = filterSize / 2 + filterSize %2;

    //for speed we predefine the memory for some matrices
    //The saliency map is bigger than the image of the maximum size among the loaded filters
    int mapWidth = sensorWidth + 2 * salMapPadding;
    int mapHeight = sensorHeight + 2 * salMapPadding;

    salMapLeft = yarp::sig::Matrix(mapHeight, mapWidth);
    salMapRight = yarp::sig::Matrix(mapHeight, mapWidth);
    orient0FeatureMap = yarp::sig::Matrix(mapHeight, mapWidth);
    orient45FeatureMap = yarp::sig::Matrix(mapHeight, mapWidth);
    orient90FeatureMap = yarp::sig::Matrix(mapHeight, mapWidth);
    orient135FeatureMap = yarp::sig::Matrix(mapHeight, mapWidth);
    orient0DOGFeatureMap = yarp::sig::Matrix(mapHeight, mapWidth);
    orient45DOGFeatureMap = yarp::sig::Matrix(mapHeight, mapWidth);
    orient90DOGFeatureMap = yarp::sig::Matrix(mapHeight, mapWidth);
    orient135DOGFeatureMap = yarp::sig::Matrix(mapHeight, mapWidth);
    activationMap = yarp::sig::Matrix(mapHeight, mapWidth);

    salMapLeft.zero();
    salMapRight.zero();
    activationMap.zero();
}


void vAttentionManager::generateOrientedGaussianFilter(yarp::sig::Matrix &filterMap, double A, double sigmaX, double sigmaY,
                                                       double theta, int &filterSize, int gaussianFilterSize, int xCenter,
                                                       int yCenter) {
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
    //Update filterSize to comply with new filter
    filterSize = max(gaussianFilterSize, filterSize);
}

void vAttentionManager::generateGaborFilter(yarp::sig::Matrix &filterMap, int gaborFilterSize, double A, double f,
                                            double sigma, double theta, int &filterSize)
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

    filterSize = max(gaborFilterSize,filterSize);

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

    outPortName = "/" + moduleName + "/orient0:o";
    check &= outOrient0Port.open(outPortName);

    outPortName = "/" + moduleName + "/orient45:o";
    check &= outOrient45Port.open(outPortName);

    outPortName = "/" + moduleName + "/orient90:o";
    check &= outOrient90Port.open(outPortName);

    outPortName = "/" + moduleName + "/orient135:o";
    check &= outOrient135Port.open(outPortName);

    outPortName = "/" + moduleName + "/DOGorient0:o";
    check &= outDOGorient0Port.open(outPortName);

    outPortName = "/" + moduleName + "/DOGorient45:o";
    check &= outDOGorient45Port.open(outPortName);

    outPortName = "/" + moduleName + "/DOGorient90:o";
    check &= outDOGorient90Port.open(outPortName);

    outPortName = "/" + moduleName + "/DOGorient135:o";
    check &= outDOGorient135Port.open(outPortName);

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
    numIterations ++;
    /* get the event queue in the vBottle bot */
    ev::vQueue q = bot.get<ev::AddressEvent>();

    q.sort(true);

    int x,y;
    for (ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {
        ev::event<ev::AddressEvent> aep = ev::getas<ev::AddressEvent>(*qi);

        if (!aep) continue;

        x = aep->getX();
        y = aep->getY();

        // --- increase energy of saliency map  --- //
        if (aep->getChannel() == 0) {
            //TODO handle left and right salMap
            //Update gabor
            updateMap(orient0FeatureMap, orient0FilterMap, x, y);
            updateMap(orient45FeatureMap, orient45FilterMap, x, y);
            updateMap(orient90FeatureMap, orient90FilterMap, x, y);
            updateMap(orient135FeatureMap, orient135FilterMap, x, y);
            //Update Gaussian
//            updateMap(orient0DOGFeatureMap, orient0DOGFilterMap, x, y, dt);
//            updateMap(orient45DOGFeatureMap, orient45DOGFilterMap, x, y, dt);
//            updateMap(orient90DOGFeatureMap, orient90DOGFilterMap, x, y, dt);
//            updateMap(orient135DOGFeatureMap, orient135DOGFilterMap, x, y, dt);
        } else {
            //TODO
        }
    }

    double t = yarp::os::Time::now();
    double dt = t - prevT;
    prevT = t;
    decayMap(orient0FeatureMap, dt);
    decayMap(orient45FeatureMap, dt);
    decayMap(orient90FeatureMap, dt);
    decayMap(orient135FeatureMap, dt);

    threshold(orient0FeatureMap, orient0FeatureMap, 0);
    threshold(orient45FeatureMap, orient45FeatureMap, 0);
    threshold(orient90FeatureMap, orient90FeatureMap, 0);
    threshold(orient135FeatureMap, orient135FeatureMap, 0);

    //Normalise Gabor
//    normaliseMap(orient0FeatureMap,normalizedOrient0FeatureMap);
//    normaliseMap(orient45FeatureMap,normalizedOrient45FeatureMap);
//    normaliseMap(orient90FeatureMap,normalizedOrient90FeatureMap);
//    normaliseMap(orient135FeatureMap,normalizedOrient135FeatureMap);

//    double th = 0.0002;
    double th = 50;
    threshold(orient0FeatureMap, threshOrient0FeatureMap, th, true);
    threshold(orient45FeatureMap, threshOrient45FeatureMap, th, true);
    threshold(orient90FeatureMap, threshOrient90FeatureMap, th, true);
    threshold(orient135FeatureMap, threshOrient135FeatureMap, th, true);


    //Normalise Gaussian
//    normaliseMap(orient0DOGFeatureMap,normalizedOrient0DOGFeatureMap);
//    normaliseMap(orient45DOGFeatureMap,normalizedOrient45DOGFeatureMap);
//    normaliseMap(orient90DOGFeatureMap,normalizedOrient90DOGFeatureMap);
//    normaliseMap(orient135DOGFeatureMap,normalizedOrient135DOGFeatureMap);


    // ---- adding the event to the output vBottle if it passes thresholds ---- //

    /*
     if(pFeaOn[posFeaImage] > thrOn) {
     std::cout << "adding On event to vBottle" << std::endl;

     ev::AddressEvent ae = *aep;
     ae.setPolarity(1);
     ae.setX(xevent);
     ae.setY(yevent);

     outBottle->addEvent(ae);

     }
     else if(pFeaOff[posFeaImage] < thrOff) {
     std::cout << "adding Off event to vBottle" << std::endl;
     ev::AddressEvent ae = *aep;
     ae.setPolarity(0);
     ae.setX(xevent);
     ae.setY(yevent);

     outBottle->addEvent(ae);

     }
    ev::vBottle &outBottle = outPort.prepare();
    outBottle.clear();
    yarp::os::Stamp st;
    this->getEnvelope(st);
    outPort.setEnvelope(st);
    // --- writing vBottle on buffered output port
    if (strictness) {
        outPort.writeStrict();
    } else {
        outPort.write();
    }
     */

    //  --- convert to images for display --- //

    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageLeft = outSalMapLeftPort.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageActivation = outActivationMapPort.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageOrient0 = outOrient0Port.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageOrient45 = outOrient45Port.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageOrient90 = outOrient90Port.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageOrient135 = outOrient135Port.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageDOGorient0 = outDOGorient0Port.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageDOGorient45 = outDOGorient45Port.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageDOGorient90 = outDOGorient90Port.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageDOGorient135 = outDOGorient135Port.prepare();

    threshOrient0FeatureMap *= 255;
    threshOrient45FeatureMap *= 255;
    threshOrient90FeatureMap *= 255;
    threshOrient135FeatureMap *= 255;

    convertToImage(threshOrient0FeatureMap, imageOrient0,salMapPadding);
    convertToImage(threshOrient45FeatureMap, imageOrient45, salMapPadding);
    convertToImage(threshOrient90FeatureMap, imageOrient90, salMapPadding);
    convertToImage(threshOrient135FeatureMap, imageOrient135, salMapPadding);

    convertToImage(orient0DOGFeatureMap, imageDOGorient0, salMapPadding);
    convertToImage(orient45DOGFeatureMap, imageDOGorient45, salMapPadding);
    convertToImage(orient90DOGFeatureMap, imageDOGorient90, salMapPadding);
    convertToImage(orient135DOGFeatureMap, imageDOGorient135, salMapPadding);

//    convertToImage(orient0FilterMap, imageOrient0,0);
//    convertToImage(orient45FilterMap, imageOrient45, 0);
//    convertToImage(orient90FilterMap, imageOrient90, 0);
//    convertToImage(orient135FilterMap, imageOrient135, 0);
//
//    convertToImage(orient0DOGFilterMap, imageDOGorient0, 0);
//    convertToImage(orient45DOGFilterMap, imageDOGorient45, 0);
//    convertToImage(orient90DOGFilterMap, imageDOGorient90, 0);
//    convertToImage(orient135DOGFilterMap, imageDOGorient135, 0);


//    salMapLeft = normalizedOrient0FeatureMap + normalizedOrient45FeatureMap + normalizedOrient90FeatureMap + normalizedOrient135FeatureMap;
//             + normalizedOrient0DOGFeatureMap + normalizedOrient45DOGFeatureMap + normalizedOrient90DOGFeatureMap + normalizedOrient135DOGFeatureMap;

    salMapLeft = threshOrient0FeatureMap + threshOrient45FeatureMap + threshOrient90FeatureMap + threshOrient135FeatureMap;
    salMapLeft /= 4;
    computeAttentionPoint(salMapLeft);

    int topX,topY,bottomX,bottomY;
    computeBoundingBox(salMapLeft, 10, attPointRow, attPointCol, topY, topX, bottomY, bottomX);

    convertToImage(salMapLeft, imageLeft, salMapPadding, attPointRow, attPointCol);
    drawBoundingBox(imageLeft, topY, topX, bottomY, bottomX);
    convertToImage(activationMap, imageActivation, salMapPadding, attPointRow, attPointCol);


    // --- writing images of left and right saliency maps on output port
    if (outSalMapLeftPort.getOutputCount()) {
        outSalMapLeftPort.write();
    }

    if (outActivationMapPort.getOutputCount()) {
        outActivationMapPort.write();
    }

    if (outOrient0Port.getOutputCount()) {
        outOrient0Port.write();
    }
    if (outOrient45Port.getOutputCount()) {
        outOrient45Port.write();
    }
    if (outOrient90Port.getOutputCount()) {
        outOrient90Port.write();
    }
    if (outOrient135Port.getOutputCount()) {
        outOrient135Port.write();
    }
    if (outDOGorient0Port.getOutputCount()) {
        outDOGorient0Port.write();
    }
    if (outDOGorient45Port.getOutputCount()) {
        outDOGorient45Port.write();
    }
    if (outDOGorient90Port.getOutputCount()) {
        outDOGorient90Port.write();
    }
    if (outDOGorient135Port.getOutputCount()) {
        outDOGorient135Port.write();
    }

}

void vAttentionManager::convertToImage(const yarp::sig::Matrix &map, yarp::sig::ImageOf<yarp::sig::PixelBgr> &image,
                                       int mapPaddingSize, int rMax, int cMax) {

    /*prepare output vBottle with images */
    int imageRows = map.rows() - mapPaddingSize;
    int imageCols = map.cols() - mapPaddingSize;
    image.resize(imageRows,imageCols);
    image.setTopIsLowIndex(true);
    image.zero();

    int shiftedR, shiftedC;
    for (int r = imageRows - 1; r > 0; r--) {
        for (int c = 0; c < imageCols; c++) {
            yarp::sig::PixelBgr pixelBgr;

//          Coordinates of saliency map are shifted by salMapPadding wrt the image
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

            image(c, imageRows - r) = pixelBgr;
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

void vAttentionManager::updateMap(yarp::sig::Matrix &map, const yarp::sig::Matrix &filterMap, int x, int y) {

    int filterRows = filterMap.rows();
    int filterCols = filterMap.cols();

    // ---- increase energy in the location of the event ---- //
    for (int rf = 0; rf < filterRows; rf++) {
        for (int cf = 0; cf < filterCols; cf++) {
            map(x + rf, y + cf) += filterMap(rf, cf);
            clamp (map(x + rf, y + cf), -2000.0, 2000.0);
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

void vAttentionManager::computeAttentionPoint(const yarp::sig::Matrix &map) {

    yarp::sig::Matrix decisionMap = map;
    int rectSize = 20;

    for (int i = -rectSize; i < rectSize; ++i) {
        clamp(i,0,map.rows());
        for (int j = -rectSize; j < rectSize; ++j) {
            clamp(j, 0,map.cols());
            decisionMap(i + attPointRow,j + attPointCol) *= 0.5;
        }
    }
    maxInMap(decisionMap);
    activationMap(attPointRow,attPointCol) += 5;
    maxInMap(activationMap);
//    activationMap *= 0.95;

}

void vAttentionManager::maxInMap(const yarp::sig::Matrix &map) {
    double max = 0;
    attPointCol = 0;
    attPointRow = 0;
    for (int r = 0; r < map.rows(); r++) {
        for (int c = 0; c < map.cols(); c++) {
            if (map(r, c) > max) {
                max = map(r, c);
                attPointRow = r;
                attPointCol = c;
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
    //TODO debug and draw bounding box
}

void vAttentionManager::drawBoundingBox(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, int topRow, int topCol,
                                        int bottomRow, int bottomCol) {
    for (int i = topCol; i < bottomCol; ++i) {
        image(i,topRow) = yarp::sig::PixelBgr(255,0,0);
    }
    for (int i = topCol; i < bottomCol; ++i) {
        image(i,bottomRow) = yarp::sig::PixelBgr(255,0,0);
    }
    for (int i = topRow; i < bottomRow; ++i) {
        image(topCol,i) = yarp::sig::PixelBgr(255,0,0);
    }
    for (int i = topRow; i < bottomRow; ++i) {
        image(bottomCol,i) = yarp::sig::PixelBgr(255,0,0);
    }

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
