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
/// \defgroup vCornerTracking vCornerTracking
/// \ingroup Modules
/// \brief tracks corner events fitting 3D lines to cluster of corner events

#ifndef __VCORNERTRACKINGMODULE__
#define __VCORNERTRACKINGMODULE__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/eventdriven/all.h>
#include <fstream>
#include <math.h>
#include <vCornerTrackingCallback.h>
#include <vCornerTrackingThread.h>

class vCornerTrackingModule : public yarp::os::RFModule
{

    //the event bottle input and output handler
    vCornerTrackingCallback      *cornertrackingcallback;
    vCornerTrackingThread        *cornertrackingthread;


public:

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();

};


#endif
//empty line to make gcc happy
