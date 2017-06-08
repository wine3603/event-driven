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

#include "cluster.h"

using namespace ev;

void cluster::initialise(int maxsize)
{

    tlast_update = 0.0;
    this->maxsize = maxsize;
    vel.first = 0.0;
    vel.second = 0.0;

}

double cluster::dist2event(ev::event<LabelledAE> evt)
{
    auto laste = is_event<LabelledAE>(this->getLastEvent());
    double dx = evt->x - laste->x;
    double dy = evt->y - laste->y;
    double dist = sqrt(dx*dx+dy*dy);

    return dist;
}

void cluster::addEvent(ev::event<LabelledAE> evt, unsigned int currt)
{
    cluster_.push_back(evt);
    tlast_update = currt;
//    std::cout << "last update at " << tlast_update << std::endl;

//    //remove the first event added
//    if(cluster_.size() > maxsize)
//        cluster_.pop_front();
}

ev::event<> cluster::getLastEvent()
{
    if(!cluster_.size()) return NULL;
    return cluster_.back();
}

void cluster::fitLine()
{
    yarp::sig::Vector meanvec(3);

    //matrices to store data
    unsigned n = cluster_.size();

//    std::cout << n << std::endl;

    yarp::sig::Matrix data(n, 3);
    yarp::sig::Matrix centreddata(n, 3);

    //matrices to store the result of SVD decomposition
    yarp::sig::Matrix U;
    yarp::sig::Vector S;
    yarp::sig::Matrix V;

    meanvec[0] = 0.0;
    meanvec[1] = 0.0;
    meanvec[2] = 0.0;

    int count = 0;
    unsigned int prevstamp;
    unsigned int currt;
    for(ev::vQueue::iterator qi = cluster_.begin(); qi != cluster_.end(); qi++)
    {
        auto cep = is_event<LabelledAE>(*qi);
//        std::cout << cep->x << " " << cep->y << " " << cep->stamp << std::endl;

        currt = cep->stamp;
//        std::cout << currt << std::endl;
        double dt = currt - prevstamp;
        if(dt < 0) {
            currt += ev::vtsHelper::max_stamp;
//            std::cout << currt << std::endl;
        }

        data[count][0] = cep->x;
        data[count][1] = cep->y;
        data[count][2] = currt;

        meanvec[0] += data[count][0];
        meanvec[1] += data[count][1];
        meanvec[2] += data[count][2];

        prevstamp = currt;
        count++;
    }
    meanvec[0] = meanvec[0]/n;
    meanvec[1] = meanvec[1]/n;
    meanvec[2] = meanvec[2]/n;

//    std::cout << std::endl;

    //center data
    for(unsigned int i = 0; i < n; i++)
    {
        centreddata[i][0] = data[i][0] - meanvec[0];
        centreddata[i][1] = data[i][1] - meanvec[1];
        centreddata[i][2] = data[i][2] - meanvec[2];
    }

//    std::cout << "computing SVD..." << std::endl;
    yarp::math::SVDJacobi(centreddata, U, S, V);

    //line parameters (a/c, b/c) will provide the velocity
    yarp::sig::Vector v;
    v = V.getCol(0);
//    std::cout << "from line fitting " << -v[0]/v[2] * 1000000 << " " << -v[1]/v[2] * 1000000 << std::endl;
    vel.first = -v[0]/v[2];
    vel.second = -v[1]/v[2];

}

bool cluster::isInTriangle(ev::event<LabelledAE> evt, unsigned int currt)
{
    yarp::sig::Vector apexToPoint(3);
    yarp::sig::Vector apexToCentre(3);

    auto apex = is_event<LabelledAE>(this->getLastEvent());

    //distance between apex and current point
    apexToPoint[0] = apex->x - evt->x;
    apexToPoint[1] = apex->y - evt->y;
    apexToPoint[2] = tlast_update - currt;

    //distance between apex and center of the triangle
    apexToCentre[0] = apex->x - 10;
    apexToCentre[1] = apex->y - 10;
    apexToCentre[2] = tlast_update;

    double dotProd = apexToPoint[0]*apexToCentre[0] + apexToPoint[1]*apexToCentre[1] + apexToPoint[2]*apexToCentre[2];
    double magApexToPoint = sqrt(apexToPoint[0]*apexToPoint[0] + apexToPoint[1]*apexToPoint[1] + apexToPoint[2]*apexToPoint[2]);
    double magApexToPoint = sqrt(apexToCentre[0]*apexToCentre[0] + apexToCentre[1]*apexToCentre[1] + apexToCentre[2]*apexToCentre[2]);

}

unsigned int cluster::getLastUpdate()
{
    return tlast_update;
}

int cluster::getClusterSize()
{
    return cluster_.size();
}

double cluster::getVx()
{
    return vel.first;
}

double cluster::getVy()
{
    return vel.second;
}

//empty line to make gcc happy
