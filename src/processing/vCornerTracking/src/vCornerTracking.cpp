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

#include "vCornerTracking.h"

#include <iomanip>

using namespace ev;

/**********************************************************/
bool vCornerTrackingModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vCornerTracking")).asString();
    yarp::os::RFModule::setName(moduleName.c_str());

    bool strict = rf.check("strict") &&
            rf.check("strict", yarp::os::Value(true)).asBool();

    /* set parameters */
    int height = rf.check("height", yarp::os::Value(128)).asInt();
    int width = rf.check("width", yarp::os::Value(128)).asInt();
    int mindistance = rf.check("mindist", yarp::os::Value(6)).asInt();
    unsigned int trefresh = rf.check("trefresh", yarp::os::Value(1000000)).asInt();

    /* create the thread and pass pointers to the module parameters */
    cornertrackingmanager = new vCornerTrackingManager(height, width, mindistance, trefresh);
    return cornertrackingmanager->open(moduleName, strict);

}

/**********************************************************/
bool vCornerTrackingModule::interruptModule()
{
    cornertrackingmanager->interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vCornerTrackingModule::close()
{
    cornertrackingmanager->close();
    delete cornertrackingmanager;
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vCornerTrackingModule::updateModule()
{
    return true;
}

/**********************************************************/
double vCornerTrackingModule::getPeriod()
{
    return 1;
}

/******************************************************************************/
//vCornerTrackingManager
/******************************************************************************/
vCornerTrackingManager::vCornerTrackingManager(int height, int width, int mindistance, unsigned int trefresh)
{
    this->height = height;
    this->width = width;
    this->mindistance = mindistance;
    this->trefresh = trefresh;
    clusterSet = new clusterPool();

}
/**********************************************************/
bool vCornerTrackingManager::open(const std::string moduleName, bool strictness)
{
    this->strictness = strictness;
    if(strictness) {
        std::cout << "Setting " << moduleName << " to strict" << std::endl;
        this->setStrict();
    }
    this->useCallback();

    std::string inPortName = "/" + moduleName + "/vBottle:i";
    bool check1 = BufferedPort<ev::vBottle>::open(inPortName);

    if(strictness) outPort.setStrict();
    std::string outPortName = "/" + moduleName + "/vBottle:o";
    bool check2 = outPort.open(outPortName);

    return check1 && check2;

}

/**********************************************************/
void vCornerTrackingManager::close()
{
    //close ports
    outPort.close();
    yarp::os::BufferedPort<ev::vBottle>::close();

    delete clusterSet;

}

/**********************************************************/
void vCornerTrackingManager::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    yarp::os::BufferedPort<ev::vBottle>::interrupt();
}

/**********************************************************/
void vCornerTrackingManager::onRead(ev::vBottle &bot)
{
    /*prepare output vBottle*/
    ev::vBottle * outBottle = 0;

    yarp::os::Stamp st;
    this->getEnvelope(st); outPort.setEnvelope(st);

    std::pair <double, double> vel;

    /*get the event queue in the vBottle bot*/
    ev::vQueue q = bot.get<LabelledAE>();

    for(ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        //current corner event
        auto cep = is_event<LabelledAE>(*qi);

        //update cluster velocity
        vel = clusterSet->update(cep);

        //create new flow event and assign to it the velocity of the current cluster
        auto fe = make_event<FlowEvent>(cep);
        fe->vx = vel.first;
        fe->vy = vel.second;

        if(!outBottle) {
            outBottle = &outPort.prepare();
            outBottle->clear();
        }
        outBottle->addEvent(fe);

    }

    if (strictness) outPort.writeStrict();
    else outPort.write();

}

/**********************************************************/

//empty line to make gcc happy
