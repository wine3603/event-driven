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

/// \defgroup Modules Modules
/// \defgroup vCorner vCorner
/// \ingroup Modules
/// \brief detects corner events using the Harris method

#ifndef __VCORNERTHREAD__
#define __VCORNERTHREAD__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/eventdriven/all.h>
#include <iCub/eventdriven/vtsHelper.h>
#include <filters.h>
#include <fstream>
#include <math.h>

class vComputeThread : public yarp::os::Thread
{
private:

    int sobelsize;
    int windowRad;
    double sigma;
    double thresh;
    ev::vQueue *patch;
    filters convolution;
    ev::event<ev::LabelledAE> cornerevt;

    bool detectcorner(int x, int y);

public:
    vComputeThread(int sobelsize, int windowRad, double sigma, double thresh);
    void setData(ev::vQueue *patch);
    ev::event<ev::LabelledAE> getResponse();
    bool threadInit() { return true; }
    void run();
    void threadRelease() {}
};

class vCornerThread : public yarp::os::Thread
{
private:

    //output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<yarp::os::Bottle> debugOut;
    yarp::os::BufferedPort<ev::vBottle> vBottleOut;

    //parameters
    unsigned int height;
    unsigned int width;
    std::string name;
    bool strict;
    int eventsInPatch;
    int channel;
    int windowRad;
    int sobelsize;
    double sigma;
    double thresh;

    std::vector<vComputeThread *> computeThreads;
    int nthreads;

    ev::hSurfThread eventhandler;
    filters convolution;
    double prevTime;

    bool detectcorner(ev::vQueue patch, int x, int y);

public:

    vCornerThread(unsigned int height, unsigned int width, std::string name, bool strict, int eventsInPatch,
                  int windowRad, int sobelsize, double sigma, double thresh, int nthreads);
    bool threadInit();
    void run();
    void threadRelease();
};


#endif
//empty line to make gcc happy
