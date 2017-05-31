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

#ifndef __CLUSTERPOOL__
#define __CLUSTERPOOL__

#include <iCub/eventdriven/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <math.h>
#include <utility>
#include <cluster.h>

class clusterPool {

private:

    //we store the pool of clusters in a vector
    std::vector <cluster> pool;

    //threshold on the distance between the event and the cluster
    int mindistance;

    //time from the last update to kill the cluster
    unsigned int trefresh;
    ev::vtsHelper unwrapper;

    //minimum number of events to fit the line
    int minevts;

    //check for the first event
    bool firstevent;

public:

    clusterPool(int mindistance, unsigned int trefresh, int minevts);

    std::pair<double, double> update(ev::event<ev::LabelledAE> evt);
    void createNewCluster(ev::event<ev::LabelledAE> evt);
    void killOldCluster(int clusterID);
    std::vector <cluster> getPool();
//    cluster getCluster(int clusterID);

};

#endif
//empty line to make gcc happy
