/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Valentina Vasco
 * email:  valentina.vasco@iit.it
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
#include "eventHistoryBuffer.h"

class vFilterDisparityManager : public yarp::os::BufferedPort<emorph::vBottle>
{
private:

    bool strictness;

    //output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<emorph::vBottle> outPort;

    int height;
    int width;
    int pos_x;
    int pos_y;
    int ts;
    double theta;
    int fRad;

    std::vector<double> disparity_vector;
    std::vector<double> phase_vector;
    std::vector<double> binocular_energy_theta;
    double even_conv_left;
    double odd_conv_left;
    std::vector<double> even_conv_right;
    std::vector<double> odd_conv_right; //Arrays of size = number of phases
    std::vector<double> final_convolution_even;
    std::vector<double> final_convolution_odd;
    std::vector<double> final_convolution;

    std::vector<double> disparity_est_theta;
    eventHistoryBuffer event_history;

    emorph::vSurface *surfaceOn; //< emorph::vSurface for on polarity events
    emorph::vSurface *surfaceOf; //< emorph::vSurface for off polarity events
    emorph::vSurface *cSurf;     // pointer to current surface (on or off)

    std::ofstream gaborResponse;

//    std::vector<double> even_conv_right;
//    std::vector<double> odd_conv_right; //Arrays of size = number of phases
//    std::vector<double> final_convolution_even;
//    std::vector<double> final_convolution_odd;
//    std::vector<double> final_convolution;

//    //estimated binocular energy for the single direction
//    std::vector<double> binocular_energy_theta;
//    std::vector<double>::iterator binocular_energy_theta_it;

//    //estimated disparity for the single direction
//    std::vector<double> disparity_est_theta;

//    //estimated flow for the single direction
//    std::vector<double> flow_est_theta;

    //FILTER PARAMETERS
    //directions
    int directions;
    std::vector<double> dir_vector;
    double dir_step;

    //phases
    int phases;
//    std::vector<double> phase_vector;
//    double phase_step;

//    //disparity
//    std::vector<double> disparity_vector;

    //size
    int kernel_size;

//    eventHistoryBuffer event_history;
    stFilters st_filters;

    //for helping with timestamp wrap around
    emorph::vtsHelper unwrapper;

    void convolveGabor();
    std::vector<double> computeEnergy();
    double computeDisparityTheta();
    std::pair<double, double> estimateDisparity();
    emorph::FlowEvent *computeFlow(emorph::AddressEvent &ae);
    void convolve(std::pair<double, double> &conv_value);
    emorph::FlowEvent *compute(std::vector<double> E);

public:
    
    vFilterDisparityManager(int height, int width, int directions, int phases, int kernel_size);

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
