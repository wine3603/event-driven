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

#include <vFeatureMap.h>

class vAttentionManager : public yarp::os::BufferedPort<ev::vBottle> {
private:
    
    
    using imagePort = yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> >;
    using eventPort = yarp::os::BufferedPort<ev::vBottle>;
    
    eventPort outPort;
    imagePort outSalMapLeftPort;
    imagePort outActivationMapPort;
    imagePort outFeatMap0;
    imagePort outFeatMap45;
    imagePort outFeatMap90;
    imagePort outFeatMap135;
    
    unsigned long int prevT;
    
    int count; //For debug purposes
    
    double tau;
    int boxWidth;
    int boxHeight;
    
    ev::vtsHelper unwrap;
    vFeatureMap eventMap;
    
    vFeatureMap activationMap;
    vFeatureMap salMapLeft;
    vFeatureMap salMapRight;
    
    //Filters
    std::vector<yarp::sig::Matrix> orientedFilters;
    
    //Feature Maps
    std::vector<vFeatureMap> orientFeatMap;
    //Thresholded Maps
    std::vector<vFeatureMap> threshFeatMap;
    
    Rectangle ROI;
    
    void computeAttentionPoint( const vFeatureMap &map, int &attPointRow, int &attPointCol );
    
    void
    generateOrientedGaussianFilter( yarp::sig::Matrix &filterMap, double A, double sigmaX, double sigmaY, double theta
                                    , int gaussianFilterSize, int xCenter, int yCenter );
    
    void generateGaborFilter( yarp::sig::Matrix &filterMap, int gaborFilterSize, double A, double f, double sigma
                              , double theta );
    
    void drawBoundingBox( yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, int topRow, int topCol, int bottomRow
                          , int bottomCol );
    
    void drawBoundingBox( yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, Rectangle ROI );

public:
    
    vAttentionManager() {};
    
    bool initialize( yarp::os::ResourceFinder &rf );
    
    bool open( const std::string moduleName, bool strictness = false );
    
    void close();
    
    void interrupt();
    
    //this is the entry point to your main functionality
    void onRead( ev::vBottle &bot );
    
    
};

class vAttentionModule : public yarp::os::RFModule {
    
    //the event bottle input and output handler
    vAttentionManager *attManager;
    yarp::os::Port handlerPort;                 // a port to handle messages

public:
    
    //the virtual functions that need to be overloaded
    virtual bool configure( yarp::os::ResourceFinder &rf );
    
    virtual bool interruptModule();
    
    virtual bool close();
    
    virtual double getPeriod();
    
    virtual bool updateModule();
    
    bool respond( const yarp::os::Bottle &command, yarp::os::Bottle &reply );
    
};


#endif
//empty line to make gcc happy
