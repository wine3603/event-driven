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

#ifndef __VATT__
#define __VATT__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/eventdriven/all.h>
#include <iCub/eventdriven/vtsHelper.h>
#include <math.h>
#include <ostream>

class vAttentionManager : public yarp::os::BufferedPort<ev::vBottle>
{
private:

    bool strictness;

    // output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<ev::vBottle> outPort; // /vBottle:o
    // output port where the output saliency map image (left) is sent
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outSalMapLeftPort;
    // output port where the output saliency map image (right) is sent
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outActivationMapPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outGabor0Port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outGabor45Port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outGabor90Port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outGabor135Port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outNegGabor0Port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outNegGabor45Port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outNegGabor90Port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outNegGabor135Port;


    int sensorWidth;
    int sensorHeight;
    int attPointRow;
    int attPointCol;
    int boxWidth;
    int boxHeight;
    int filterSize;
    int salMapPadding;
    int numIterations;
    double tau;
    unsigned long int prevT;

    double thrSal; // threshold to put a maximum on the saliency map (whatever is above thr will be set to the maximum value)
    double normSal;

    ev::vtsHelper unwrap;
    yarp::sig::Matrix eventMap;

    yarp::sig::Matrix activationMap;
    yarp::sig::Matrix salMapLeft;
    yarp::sig::Matrix salMapRight;

    //Filters
    yarp::sig::Matrix negGabor0;
    yarp::sig::Matrix negGabor45;
    yarp::sig::Matrix negGabor90;
    yarp::sig::Matrix negGabor135;
    yarp::sig::Matrix gabor0;
    yarp::sig::Matrix gabor45;
    yarp::sig::Matrix gabor90;
    yarp::sig::Matrix gabor135;

    //Feature Maps
    yarp::sig::Matrix gabor0FeatureMap;
    yarp::sig::Matrix gabor45FeatureMap;
    yarp::sig::Matrix gabor90FeatureMap;
    yarp::sig::Matrix gabor135FeatureMap;
    yarp::sig::Matrix negGabor0FeatureMap;
    yarp::sig::Matrix negGabor45FeatureMap;
    yarp::sig::Matrix negGabor90FeatureMap;
    yarp::sig::Matrix negGabor135FeatureMap;

    //Thresholded feature maps
    yarp::sig::Matrix threshGabor0FeatureMap;
    yarp::sig::Matrix threshGabor45FeatureMap;
    yarp::sig::Matrix threshGabor90FeatureMap;
    yarp::sig::Matrix threshGabor135FeatureMap;
    yarp::sig::Matrix threshNegGabor0FeatureMap;
    yarp::sig::Matrix threshNegGabor45FeatureMap;
    yarp::sig::Matrix threshNegGabor90FeatureMap;
    yarp::sig::Matrix threshNegGabor135FeatureMap;


    void decayMap (yarp::sig::Matrix& map, double dt);

    void updateMap(yarp::sig::Matrix &map, const yarp::sig::Matrix &filterMap, int row, int col, int topRow,
                       int topCol, int bottomRow, int bottomCol);

    void normaliseMap(const yarp::sig::Matrix &map, yarp::sig::Matrix &normalisedMap);

    void printMap(const yarp::sig::Matrix &map);

    void convertToImage(const yarp::sig::Matrix &map, yarp::sig::ImageOf<yarp::sig::PixelBgr> &image,
                        int mapPaddingSize = 0, int rMax = -1, int cMax = -1);

    void load_filter(const std::string filename, yarp::sig::Matrix &filterMap, int &filterSize);

    void computeAttentionPoint(const yarp::sig::Matrix &map, int &attPointRow, int &attPointCol);

    void generateOrientedGaussianFilter(yarp::sig::Matrix &filterMap, double A, double sigmaX, double sigmaY,
                                            double theta, int gaussianFilterSize, int xCenter, int yCenter);

    void generateGaborFilter(yarp::sig::Matrix &filterMap, int gaborFilterSize, double A, double f,
                                 double sigma, double theta);

    void drawSquare( yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, int r, int c, yarp::sig::PixelBgr &pixelBgr) ;

    void computeBoundingBox(const yarp::sig::Matrix &map, double threshold, int centerRow, int centerCol,
                                int &topRow, int &topCol, int &bottomRow, int &bottomCol);

    bool drawBoundingBox(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, int topRow, int topCol,
                         int bottomRow, int bottomCol);

    void maxInMap(const yarp::sig::Matrix &map, int &rowMax, int &colMax);

    void convolve(const yarp::sig::Matrix &featureMap, yarp::sig::Matrix &convolvedFeatureMap,
                  const yarp::sig::Matrix &filterMap);

    void threshold(const yarp::sig::Matrix &map, yarp::sig::Matrix &thresholdedMap, double threshold, bool binary = false);

    template <typename T>
    void clamp(T &val, T min, T max);

    bool energyInArea(const yarp::sig::Matrix &map, int topRow, int topCol, int bottomRow, int bottomCol,
                          double &energy);

public:

    vAttentionManager(int sensorWidth, int sensorHeight, double tau, double thrSal,
                          std::string &filtersPath);

    bool    open(const std::string moduleName, bool strictness = false);
    void    close();
    void    interrupt();

    //this is the entry point to your main functionality
    void    onRead(ev::vBottle &bot);



};

class vAttentionModule : public yarp::os::RFModule
{

    //the event bottle input and output handler
    vAttentionManager      *attManager;
    yarp::os::Port          handlerPort;                 // a port to handle messages

public:

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

};


#endif
//empty line to make gcc happy
