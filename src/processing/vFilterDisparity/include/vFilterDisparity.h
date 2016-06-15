/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Valentina Vasco and Yeshasvi Tirupachuri
 * email:  valentina.vasco@iit.it, Yeshasvi.Tirupachuri@iit.it
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

#ifndef __VFILTERDISPARITY__
#define __VFILTERDISPARITY__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/emorph/all.h>
#include <iCub/emorph/vtsHelper.h>
#include <iostream>
#include <fstream>
#include "stFilters.h"

class vFilterDisparityManager : public yarp::os::BufferedPort<emorph::vBottle>
{
private:

    bool strictness;

    //output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<emorph::vBottle> outPort;
    yarp::os::BufferedPort<yarp::os::Bottle> disparityPort;

    //for helping with timestamp wrap around
    emorph::vtsHelper unwrapper;

    int x;
    int y;
    double ts;

    //first in first out structure
    emorph::vQueue FIFO;

    int height;
    int width;
    uint nevents;
    double threshold;

    //list of tuned disparities
    yarp::os::Bottle disparitylist;

    //filters parameters
    int orientations; // number of orientations
    std::vector<double> ori_vector; // vector of orientations
    int phases; // number of phases
    std::vector<double> phase_vector; // vector of phases
    int *disparity_vector;
    int winsize;
    stFilters st_filters;

    //filters responses
    double even_conv_left;
    double odd_conv_left;
    std::vector<double> even_conv_right;
    std::vector<double> odd_conv_right; //arrays of size = number of phases
    std::vector<double> energy;

    std::ofstream outDisparity;
    std::ofstream gaborResponse;

    void computeMonocularEnergy(double theta);
    double computeBinocularEnergy();

public:
    
    vFilterDisparityManager(int height, int width, int nevents, int orientations,
                            int winsize, double threshold, yarp::os::Bottle disparitylist);

    bool    open(const std::string moduleName, bool strictness = false);
    void    close();
    void    interrupt();

    //this is the entry point to your main functionality
    void    onRead(emorph::vBottle &bot);

};

class vFilterDisparityModule : public yarp::os::RFModule
{

    //the event bottle input and output handler
    vFilterDisparityManager      *disparitymanager;


public:

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual bool close();
    virtual bool respond(const yarp::os::Bottle &command,
                         yarp::os::Bottle &reply);
    virtual double getPeriod();
    virtual bool updateModule();

};


#endif
//empty line to make gcc happy
