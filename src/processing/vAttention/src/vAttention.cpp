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

    /* attach a port of the same name as the module (prefixed with a /) to the module
     so that messages received from the port are redirected to the respond method */

    std::string handlerPortName = "/";
    handlerPortName += getName();         // use getName() rather than a literal

    if (!handlerPort.open(handlerPortName.c_str())) {
        std::cout << getName() << ": Unable to open port " << handlerPortName << std::endl;
        return false;
    }

    attach(handlerPort);                  // attach to port
    attManager = new vAttentionManager();
    return attManager->initialize(rf);

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
    std::string inPortName = moduleName + "/vBottle:i";
    return yarp::os::BufferedPort<ev::vBottle>::open(inPortName);
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

    double ampl;

    double upBound = 2000, lowBound = -2000;
    for (int r = 0; r < eventMap.rows(); ++r) {
        for (int c = 0; c < eventMap.cols(); ++c) {
            ampl = (double) eventMap(r, c);
            if (ampl != 0) {
                for (unsigned int i = 0; i < orientFeatMap.size();++i) {
                            orientFeatMap[i].updateWithFilter(orientedFilters[i],r,c,upBound,lowBound);
                }
            }
        }
    }

    unsigned long int t = unwrap (q.back().get()->getStamp());

    double dt = (double)(t - prevT) * (80*pow(10,-9));
    prevT = t;
    double th = 50;
    bool binary = false;

    for (unsigned int j = 0; j < orientFeatMap.size(); ++j) {
        orientFeatMap[j].decay(dt,tau);
        orientFeatMap[j].threshold(th, binary, &threshFeatMap[j]);
        yarp::sig::ImageOf<yarp::sig::PixelBgr> &image = outFeatMapPort[j]->prepare();
        threshFeatMap[j].convertToImage(image);
        outFeatMapPort[j]->write();
        salMapLeft += threshFeatMap[j];
    }
    activationMap.decay(dt,tau);

    salMapLeft.normalise();
    salMapLeft*= 200000;

    //  --- convert to images for display --- //
    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageLeft = outSalMapLeftPort.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelBgr> &imageActivation = outActivationMapPort.prepare();

    activationMap.convertToImage(imageActivation);
    salMapLeft.convertToImage(imageLeft);

    outSalMapLeftPort.write();
    outActivationMapPort.write();

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
//            if (inIOR) {
//                map(rMap, cMap) *= 0.5;
//            } else {
                map(rMap, cMap) += filterMap(rFil, cFil);
                clamp(map(row + rFil, col + cFil), -2000.0, 2000.0);
//            }
        }
    }

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

    while ((internalEnergy - previousInternalEnergy) / internalEnergy > threshold){
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

template <typename T>
void vAttentionManager::clamp(T &val, T min, T max) {
    val = std::max (min, val);
    val = std::min (max, val);
}

bool vAttentionManager::initialize(yarp::os::ResourceFinder &rf) {

    /* set parameters */
    int sensorWidth = rf.check("sensorWidth", yarp::os::Value(304)).asInt();
    int sensorHeight = rf.check("sensorHeight", yarp::os::Value(240)).asInt();
    tau = rf.check("tau", yarp::os::Value(3.0)).asDouble();
    boxWidth = rf.check("boxWidth", yarp::os::Value(2)).asInt();
    boxHeight = rf.check("boxHeight", yarp::os::Value(2)).asInt();
    int filterSize = rf.check("filterSize", yarp::os::Value(15)).asInt();
    bool strictness = rf.check("strict", yarp::os::Value(false)).asBool();
    std::string moduleName = rf.check("name", yarp::os::Value("vAttention")).asString();


    int salMapPadding = filterSize / 2 + filterSize %2;
    int mapWidth = sensorWidth / boxWidth;
    int mapHeight = sensorHeight / boxHeight;


    bool check = true;
    moduleName = "/" + moduleName;
    check &= open(moduleName,strictness);
    check &= outActivationMapPort.open(moduleName + "/activationMap:o");
    check &= outSalMapLeftPort.open(moduleName + "/salMapLeft:o");

    double sigmaGabor =(double) filterSize / 4.0;
    double fGabor = 2.0 /(double) filterSize;
    double amplGabor = 0.4;
    int step = 45;
    for (int theta = 0; theta < 180; theta += step) {
        vFeatureMap featureMap (mapHeight, mapWidth, salMapPadding, salMapPadding);
        yarp::sig::Matrix filterMap;
        yarp::os::BufferedPort<yarp::sig::ImageOf< yarp::sig::PixelBgr> > port;
        std::string portName = moduleName + "/featMap" + std::to_string(theta) + ":o";
        check &= port.open(portName);
        generateGaborFilter(filterMap,filterSize,amplGabor,fGabor, sigmaGabor, theta);
        orientFeatMap.push_back(featureMap);
        orientedFilters.push_back(filterMap);
        outFeatMapPort.push_back(&port); //TODO
    }

    eventMap = vFeatureMap(sensorHeight / boxHeight, sensorWidth / boxWidth);
    salMapLeft = vFeatureMap(mapHeight, mapWidth, salMapPadding, salMapPadding);
    salMapRight = vFeatureMap(mapHeight, mapWidth, salMapPadding, salMapPadding);
    activationMap = vFeatureMap(mapHeight, mapWidth, salMapPadding, salMapPadding);

    return check;
}


//empty line to make gcc happy
