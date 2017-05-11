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

#ifndef __LOCALQUEUE__
#define __LOCALQUEUE__

#include <iCub/eventdriven/all.h>
#include <yarp/sig/all.h>

using namespace ev;

class localQueue {

private:

    //parameters
    unsigned int eventsInPatch;
    unsigned int patchEdgeLength;

    //local storage
    vQueue q;
    yarp::sig::Matrix spatial;

    //internal variables
    int xoffset;
    int yoffset;

public:

    localQueue() {}

    void initialise(unsigned int eventsInPatch, int patchEdgeLength, int xCentre, int yCentre)
    {
        int srad = (patchEdgeLength - 1)/2;
        this->eventsInPatch = eventsInPatch;
        this->patchEdgeLength = patchEdgeLength;
        this->xoffset = xCentre - srad;
        this->yoffset = yCentre - srad;
        spatial.resize(patchEdgeLength, patchEdgeLength);

    }

    void addEvent(event<AE> v)
    {
        q.push_back(v);
    }

    vQueue getSurface()
    {

        vQueue qcopy;
        spatial.zero();
        int needed = 0;

        vQueue::reverse_iterator qi = q.rbegin();
        while(qi != q.rend() && qcopy.size() < eventsInPatch) {

            auto v = is_event<AE>(*qi);
            double &surfaceelement = spatial(v->x - xoffset, v->y - yoffset);

            if(surfaceelement == 0) {
                surfaceelement = 1;
                qcopy.push_back(v);
            }

            qi++;
            needed++;
        }

        //here needed is the number of events that we need to keep a surface of
        //qlength
        while(q.size() > needed)
            q.pop_front();

        return qcopy;

    }

    event<> getMostRecent()
    {
        if(!q.size()) return NULL;
        return q.back();
    }

    void print(yarp::sig::Matrix mat)
    {
        std::cout << "mat " << std::endl;
        for(int i = 0; i < mat.rows(); i++){
            for(int j = 0; j < mat.cols(); j++) {
                std::cout << mat(i, j) << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl << std::endl;
    }

};


#endif
//empty line to make gcc happy
