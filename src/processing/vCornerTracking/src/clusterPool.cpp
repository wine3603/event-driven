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

#include "clusterPool.h"

using namespace ev;

clusterPool::clusterPool(int mindistance, unsigned int trefresh, int minevts)
{
    this->mindistance = mindistance; //px
    this->trefresh = trefresh; //s
    firstevent = true;
    this->minevts = minevts;
}

std::pair <double, double> clusterPool::update(ev::event<ev::LabelledAE> evt)
{
    double dist = 0.0, mindist = mindistance;
    int clusterID = -1;
    std::pair <double, double> clustervel;

//    std::cout << "curr event " << evt->x << " " << evt->y << std::endl;

    //if it's the first event, we create the first cluster
    if(firstevent) {
        firstevent = false;
        createNewCluster(evt);
    }

    //compute the distance from the current event to each cluster
    for(unsigned int i = 0; i < pool.size(); i++) {
        dist = pool[i].dist2event(evt);

        //compute the minimum distance from all clusters
        if(dist < mindist) {
            mindist = dist;
            clusterID = i;
        }

        //check for old clusters
        if( (evt->stamp - pool[i].getLastUpdate()) > trefresh ) {
//            std::cout << evt->stamp << " " << pool[i].getLastUpdate() << std::endl;
            killOldCluster(i);
        }

    }

    //if the minimum distance is less than a predefined threshold
    if(clusterID >= 0) {

        //add corner event to the cluster
//        std::cout << evt->x << " " << evt->y << " " << evt->stamp << std::endl;
        pool[clusterID].addEvent(evt);

        //fit line to the cluster
        if(pool[clusterID].getClusterSize() > minevts) {
//            std::cout << "fitting line to cluster " << clusterID << " to event " << evt->x << " " << evt->y << " " << evt->stamp << std::endl;
            pool[clusterID].fitLine();
            clustervel.first = pool[clusterID].getVx();
            clustervel.second = pool[clusterID].getVy();
        }
        else {
            clustervel.first = 0.0;
            clustervel.second = 0.0;
        }

//        std::cout << pool[clusterID].getVx() << " " << pool[clusterID].getVy() << std::endl;

    } else {

        //create new cluster
        createNewCluster(evt);
        clusterID = pool.size() + 1;
    }

    //return the velocity of the current cluster
    return clustervel;

}

void clusterPool::createNewCluster(ev::event<ev::LabelledAE> evt)
{
    cluster newcluster;
    newcluster.addEvent(evt);
    pool.push_back(newcluster);
}

void clusterPool::killOldCluster(int clusterID)
{
    pool.erase(pool.begin() + clusterID);
}



//std::vector <cluster> clusterPool::getPool()
//{
//    return pool;
//}

//empty line to make gcc happy
