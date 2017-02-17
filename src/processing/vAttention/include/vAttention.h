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
#include <iCub/emorph/all.h>
#include <iCub/emorph/vtsHelper.h>
#include <math.h>
#include <ostream>


class vAttentionManager : public yarp::os::BufferedPort<emorph::vBottle>
{
private:

    bool strictness;

    // output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<emorph::vBottle> outPort; // /vBottle:o
    // output port where the output saliency map image (left) is sent
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outSalMapLeftPort;
    // output port where the output saliency map image (right) is sent
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outActivationMapPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outOrient0Port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outOrient45Port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outOrient90Port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outOrient135Port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outDOGorient0Port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outDOGorient45Port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outDOGorient90Port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outDOGorient135Port;


    int sensorSize;
    int filterSize;
    int attPointY;
    int attPointX;
    int salMapPadding;
    int numIterations;
    double tau;
    double prevT;
    double thrSal; // threshold to put a maximum on the saliency map (whatever is above thr will be set to the maximum value)
    double normSal;

    emorph::vtsHelper unwrap;

    yarp::sig::Matrix activationMap;
    yarp::sig::Matrix salMapLeft;
    yarp::sig::Matrix salMapRight;

    //Filters
    yarp::sig::Matrix orient0DOGFilterMap;
    yarp::sig::Matrix orient45DOGFilterMap;
    yarp::sig::Matrix orient90DOGFilterMap;
    yarp::sig::Matrix orient135DOGFilterMap;
    yarp::sig::Matrix orient0FilterMap;
    yarp::sig::Matrix orient45FilterMap;
    yarp::sig::Matrix orient90FilterMap;
    yarp::sig::Matrix orient135FilterMap;

    //Feature Maps
    yarp::sig::Matrix orient0FeatureMap;
    yarp::sig::Matrix orient45FeatureMap;
    yarp::sig::Matrix orient90FeatureMap;
    yarp::sig::Matrix orient135FeatureMap;
    yarp::sig::Matrix orient0DOGFeatureMap;
    yarp::sig::Matrix orient45DOGFeatureMap;
    yarp::sig::Matrix orient90DOGFeatureMap;
    yarp::sig::Matrix orient135DOGFeatureMap;

    //Normalized feature maps
    yarp::sig::Matrix normalizedOrient0FeatureMap;
    yarp::sig::Matrix normalizedOrient45FeatureMap;
    yarp::sig::Matrix normalizedOrient90FeatureMap;
    yarp::sig::Matrix normalizedOrient135FeatureMap;
    yarp::sig::Matrix normalizedOrient0DOGFeatureMap;
    yarp::sig::Matrix normalizedOrient45DOGFeatureMap;
    yarp::sig::Matrix normalizedOrient90DOGFeatureMap;
    yarp::sig::Matrix normalizedOrient135DOGFeatureMap;

    //Thresholded feature maps
    yarp::sig::Matrix threshOrient0FeatureMap;
    yarp::sig::Matrix threshOrient45FeatureMap;
    yarp::sig::Matrix threshOrient90FeatureMap;
    yarp::sig::Matrix threshOrient135FeatureMap;


    void decayMap (yarp::sig::Matrix& map, double dt);

    void updateMap(yarp::sig::Matrix &map, const yarp::sig::Matrix &filterMap, int x, int y);

    void normaliseMap(const yarp::sig::Matrix &map, yarp::sig::Matrix &normalisedMap);

    void printMap(const yarp::sig::Matrix &map);

    void convertToImage(const yarp::sig::Matrix &map, yarp::sig::ImageOf<yarp::sig::PixelBgr> &image,
                            int mapPaddingSize = 0, int rMax = -1, int cMax = -1);

    void load_filter(const std::string filename, yarp::sig::Matrix &filterMap, int &filterSize);

    void computeAttentionPoint(const yarp::sig::Matrix &map);

    void generateOrientedGaussianFilter(yarp::sig::Matrix &filterMap, double A, double sigmaX, double sigmaY,
                                            double theta, int &filterSize, int gaussianFilterSize, int xCenter = 0,
                                            int yCenter = 0);

    void generateGaborFilter(yarp::sig::Matrix &filterMap, int gaborFilterSize, double A, double f,
                             double sigma, double theta, int &filterSize);

    void drawSquare( yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, int r, int c, yarp::sig::PixelBgr &pixelBgr) ;

    void computeBoundingBox(const yarp::sig::Matrix &map, double threshold);

    void maxInMap(const yarp::sig::Matrix &map);

    void convolve(const yarp::sig::Matrix &featureMap, yarp::sig::Matrix &convolvedFeatureMap,
                  const yarp::sig::Matrix &filterMap);

    void threshold(const yarp::sig::Matrix &map, yarp::sig::Matrix &thresholdedMap, double threshold, bool binary = false);

    template <typename T>
    void clamp(T &val, T min, T max);

public:

    vAttentionManager(int sensorSize, double tau, double thrSal, std::string &filtersPath);

    bool    open(const std::string moduleName, bool strictness = false);
    void    close();
    void    interrupt();

    //this is the entry point to your main functionality
    void    onRead(emorph::vBottle &bot);



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
