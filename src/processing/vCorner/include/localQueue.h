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

    vQueue q;
    unsigned int qlength;
    yarp::sig::Matrix spatial;
    int slength;
    int srad;

public:

    localQueue()
    {

    }

    ~localQueue()
    {

    }

    void initLocalQueue(unsigned int qlen, int slen)
    {
        this->qlength = qlen;
        this->slength = slen;
        spatial.resize(slength, slength);
        srad = (slen - 1)/2;
    }

    void addEvent(event<> v)
    {

        //add event to the queue
        q.push_back(v);

        //if queue has more than qlength events
        //remove events
        if(q.size() > qlength)
        {
            q.pop_front();
        }
    }

    vQueue getLocalPatch()
    {
        event<AddressEvent> v;
        vQueue qcopy;

        event<AddressEvent> vfirst = as_event<AddressEvent>(*q.rbegin());

        //go through the queue
        for(vQueue::reverse_iterator qi = q.rbegin(); qi != q.rend(); qi++) {
            v = as_event<AddressEvent>(*qi);

            int dx = vfirst->x - v->x;
            int dy = vfirst->y - v->y;

//            std::cout << vfirst->x << " " << vfirst->y << " " << v->x << " " << v->y << " " <<
//                         dx + srad << " " << dy + srad << " " << spatial(dx + srad, dy + srad) << std::endl;

            //if window is empty at this location
            if(spatial(dx + srad, dy + srad) == 0) {
                spatial(dx + srad, dy + srad) = 1;

                //add event
                qcopy.push_back(v);

            }
        }

//        print(spatial);

        //reset window
        spatial.zero();

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

    int getSize()
    {
        return q.size();
    }

};


#endif
//empty line to make gcc happy
