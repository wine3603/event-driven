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

clusterPool::clusterPool(int mindistance, unsigned int trefresh, int maxsize, int minevts)
{
    this->mindistance = mindistance; //px
    this->trefresh = trefresh; //s
    firstevent = true;
    this->maxsize = maxsize; //number of events
    this->minevts = minevts;
}

std::pair <double, double> clusterPool::update(ev::event<ev::LabelledAE> evt, unsigned int currt)
{
    double dist = 0.0, mindist = mindistance;
    int clusterID = -1;
    std::pair <double, double> clustervel;

    //if it's the first event, we create the first cluster
    if(firstevent) {
        firstevent = false;
        createNewCluster(evt, currt);
    }

    //compute the distance from the current event to each cluster
    for(unsigned int i = 0; i < pool.size(); i++) {
        dist = pool[i].dist2event(evt);

        //compute the minimum distance from all clusters
        if(dist < mindist) {
            mindist = dist;
            clusterID = i;
            pool[clusterID].addEvent(evt, currt);
//            std::cout << "updating " << clusterID << " at time " << currt << std::endl;
        }
    }

    clustervel.first = 0.0;
    clustervel.second = 0.0;
    //if the minimum distance is less than a predefined threshold
    if(clusterID >= 0) {

//        auto last = ev::is_event<ev::LabelledAE>(pool[clusterID].getLastEvent());
//        int dx = evt->x - last->x;
//        int dy = evt->y - last->y;
//        double distance = sqrt(dx*dx + dy*dy);

        //add corner event to the cluster
        //        std::cout << evt->x << " " << evt->y << " " << evt->stamp << std::endl;
//        pool[clusterID].addEvent(evt, currt);

//        //fit line to the cluster
//        if(distance < 15.0 && distance > 3.0) {
//            pool[clusterID].fitLine();
//            clustervel.first = pool[clusterID].getVx();
//            clustervel.second = pool[clusterID].getVy();
//        }

//        if(distance > 15.0)
//            killOldCluster(clusterID);

        if(pool[clusterID].getClusterSize() > minevts) {
            //            std::cout << "fitting line to cluster " << clusterID << " to event " << evt->x << " " << evt->y << " " << evt->stamp << std::endl;
            pool[clusterID].fitLine();
            clustervel.first = pool[clusterID].getVx();
            clustervel.second = pool[clusterID].getVy();
        }

        //        std::cout << pool[clusterID].getVx() << " " << pool[clusterID].getVy() << std::endl;

    } else {

        //create new cluster
        createNewCluster(evt, currt);
        clusterID = pool.size() + 1;
//        std::cout << "creating new cluster " << clusterID << std::endl;
    }

    //check for old clusters
    for(unsigned int i = 0; i < pool.size(); i++) {
//        std::cout << "for cluster " << i << " " << " last update at " << pool[i].getLastUpdate() << " " << currt - pool[i].getLastUpdate() << std::endl;
        if( (currt - pool[i].getLastUpdate()) > trefresh) {
//            std::cout << " killing cluster " << i << " " << pool.size() << std::endl;
            killOldCluster(i);
        }
    }
//    std::cout << std::endl;

    //return the velocity of the current cluster
    return clustervel;

}

std::pair <double, double> clusterPool::updateNew(ev::event<ev::LabelledAE> evt, unsigned int currt)
{
    int clusterID = -1;

    //if it's the first event, we create the first cluster
    if(firstevent) {
        firstevent = false;
        createNewCluster(evt, currt);
    }

    //for each created cluster
    for(unsigned int i = 0; i < pool.size(); i++) {

        //discard the event if it's already in the cluster


        //if the event is within the triangle add it
        //CHECK IF THIS HAPPENS FOR MORE CLUSTERS
        if(pool[i].isInTriangle(evt)) {
            clusterID = i;
            pool[clusterID].addEvent(evt, currt);
        }

    }

    if(clusterID >= 0) {
        if(pool[clusterID].getClusterSize() > minevts) {
            pool[clusterID].fitLine();
            clustervel.first = pool[clusterID].getVx();
            clustervel.second = pool[clusterID].getVy();
        }

    } else {
        //create new cluster
        createNewCluster(evt, currt);
        clusterID = pool.size() + 1;
    }


}

void clusterPool::createNewCluster(ev::event<ev::LabelledAE> evt, unsigned int currt)
{
    cluster newcluster;
    newcluster.initialise(maxsize);
    newcluster.addEvent(evt, currt);
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
