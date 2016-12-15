/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Arren.Glover@iit.it
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

#include "autoSaccade.h"
#include <math.h>

/**********************************************************/
void saccadeModule::generateTrajectory(){
    double cx = 160, cy = 120, r = 10;
    double step = M_PI/18;
    for (double i = 0; i < 2*M_PI; i+= step){
        yarp::sig::Vector circlePoint(2);
        circlePoint[0] = (int)(cx + r * cos(i));
        circlePoint[1] = (int)(cy + r * sin(i));
        trajectory.push_back(circlePoint);
    }
    for (int i = 0; i < trajectory.size(); i++){
        std::cout << "point " << i << " = " << trajectory[i].toString().c_str() << std::endl;
    }
}

/**********************************************************/
bool saccadeModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("autoSaccade")).asString();
    setName(moduleName.c_str());

    //open and attach the rpc port
    std::string rpcPortName  =  "/" + moduleName + "/rpc:i";

    if (!rpcPort.open(rpcPortName))
    {
        std::cerr << getName() << " : Unable to open rpc port at " <<
                     rpcPortName << std::endl;
        return false;
    }

    //make the respond method of this RF module respond to the rpcPort
    attach(rpcPort);

    std::string condev = rf.check("robot", yarp::os::Value("none")).asString();
    yarp::os::Property options;
    options.put("device", "gazecontrollerclient");
    options.put("local", "/" + moduleName);
    //options.put("remote", "/" + condev + "/head");
//    options.put("remote","/icubSim/head");
    options.put("remote","/iKinGazeCtrl");
//    pc = 0;
    ec = 0;

    mdriver.open(options);
    if(!mdriver.isValid())
        std::cerr << "Did not connect to robot/simulator" << std::endl;
    else {
//        mdriver.view(pc);
        mdriver.view(gazeControl);
        mdriver.view(ec);
    }

    generateTrajectory();

    gazeControl->blockNeckPitch();
    gazeControl->blockNeckRoll();
    gazeControl->blockNeckYaw();


//    if(!pc)
//        std::cerr << "Did not connect to position control" << std::endl;
//    else {
//        int t; pc->getAxes(&t);
//        std::cout << "Number of Joints: " << t << std::endl;
//    }

    if(!gazeControl)
        std::cerr << "Did not connect to gaze controller" << std::endl;

    if(!ec)
       std::cerr << "Did not connect to encoders" << std::endl;
    else {
        int t; ec->getAxes(&t);
        std::cout << "Number of Joints: " << t << std::endl;
    }


    //set other variables we need from the
    checkPeriod = rf.check("checkPeriod",
                           yarp::os::Value(0.5)).asDouble();
    minVpS = rf.check("minVpS", yarp::os::Value(800)).asDouble();
    prevStamp = 4294967295; //max value

    sMag = rf.check("sMag", yarp::os::Value(2)).asDouble();
    sVel = rf.check("sVel", yarp::os::Value(1000)).asDouble();

    if(sMag < -5) {
        std::cout << "Hard-coded limit of sMag < 5" << std::endl;
        sMag = -5;
    }

    if(sMag > 5) {
        std::cout << "Hard-coded limit of sMag < 5" << std::endl;
        sMag = 5;
    }

    if(sVel < 0) {
        std::cout << "Hard-coded limit of sVel > 10" << std::endl;
        sVel = 10;
    }

    if(sVel > 1000) {
        std::cout << "Hard-coded limit of sMag < 1000" << std::endl;
        sVel = 1000;
    }

    eventBottleManager.open(moduleName);

    return true ;
}

/**********************************************************/
bool saccadeModule::interruptModule()
{
    std::cout << "Interrupting" << std::endl;
    rpcPort.interrupt();
    eventBottleManager.interrupt();
    std::cout << "Finished Interrupting" << std::endl;
    return true;
}

/**********************************************************/
bool saccadeModule::close()
{

    std::cout << "Closing" << std::endl;
    rpcPort.close();
    eventBottleManager.close();
//    delete pc;
    delete gazeControl;
    delete ec;
    mdriver.close();
    std::cout << "Finished Closing" << std::endl;
    return true;
}


void saccadeModule::performSaccade()
{
    if(!gazeControl) {
        std::cerr << "Saccade cannot be performed" << std::endl;
        return;
    }

    for (int i = 0; i < trajectory.size(); i++) {
        yarp::sig::Vector px = trajectory[i];
        gazeControl->lookAtMonoPixel(0, px);
        gazeControl->waitMotionDone();
    }

//    double vel3, vel4;
//    pc->getRefSpeed(3, &vel3);
//    pc->getRefSpeed(4, &vel4);
//
//    pc->setRefSpeed(3, sVel);
//    pc->setRefSpeed(4, sVel);

    double pos3, pos4;
    ec->getEncoder(3, &pos3);
    ec->getEncoder(4, &pos4);



  /*

    //move up
    bool movedone = false;
    pc->positionMove(3, pos3 + sMag);
    while(!movedone)
        pc->checkMotionDone(3, &movedone);

    //move down
    movedone = false;
    pc->positionMove(3, pos3 - sMag);
    while(!movedone)
        pc->checkMotionDone(3, &movedone);

    //move back up
    movedone = false;
    pc->positionMove(3, pos3);
    while(!movedone)
        pc->checkMotionDone(3, &movedone);

    //move left
    movedone = false;
    pc->positionMove(4, pos4 + sMag);
    while(!movedone)
        pc->checkMotionDone(4, &movedone);

    //move right
    movedone = false;
    pc->positionMove(4, pos4 - sMag);
    while(!movedone)
        pc->checkMotionDone(4, &movedone);

    //move back
    movedone = false;
    pc->positionMove(4, pos4);
    while(!movedone)
        pc->checkMotionDone(4, &movedone);

    pc->setRefSpeed(3, vel3);
    pc->setRefSpeed(4, vel4);

*/
}

/*
bool saccadeModule::updateModule()
{
    //if there is no connection don't do anything yet
    if(!eventBottleManager.getInputCount()) return true;

    //check the last time stamp and count
    unsigned long int latestStamp = eventBottleManager.getTime();
    unsigned long int vCount = 1000000 * eventBottleManager.popCount();

//    double latestStamp = yarp::os::Time::now();

//    std::cout << "latestStamp Update= " << latestStamp << std::endl;
//    std::cout << "prevStamp = " << prevStamp << std::endl;
    std::cout << "vCount = " << vCount << std::endl;

    double vPeriod = latestStamp - prevStamp;

    prevStamp = latestStamp;

    //this should only occur on first bottle to initialise
    if(vPeriod < 0) return true;

    if(vPeriod == 0 || (vCount / vPeriod) < minVpS) {
        //perform saccade
        if(gazeControl && ec) performSaccade();
        std::cout << "perform saccade: ";
    }
    std::cout << vPeriod/1000000 << "s | " << vCount/vPeriod
              << " v/s" << std::endl;

    return true;
}
*/

bool saccadeModule::updateModule() {
    performSaccade();
}
double saccadeModule::getPeriod()
{
    return checkPeriod;
}

bool saccadeModule::respond(const yarp::os::Bottle &command,
                              yarp::os::Bottle &reply)
{
    //fill in all command/response plus module update methods here
    return true;
}

/**********************************************************/
EventBottleManager::EventBottleManager()
{

    //here we should initialise the module
    vCount = 0;
    latestStamp = 0;

    
}
/**********************************************************/
bool EventBottleManager::open(const std::string &name)
{
    //and open the input port

    this->useCallback();

    std::string inPortName = "/" + name + "/vBottle:i";
    yarp::os::BufferedPort<emorph::vBottle>::open(inPortName);

    return true;
}

/**********************************************************/
void EventBottleManager::onRead(emorph::vBottle &bot)
{
    //create event queue
    emorph::vQueue q = bot.getSorted<emorph::AddressEvent>();
    //create queue iterator
    emorph::vQueue::iterator qi;
    
    // get the event queue in the vBottle bot
    //bot.getSorted<emorph::AddressEvent>(q);
    if(q.empty()) return;

    latestStamp = unwrapper(q.back()->getStamp());
//    std::cout << "latestStamp = " << latestStamp << std::endl;
//    std::cout << "q.size() = " << q.size() << std::endl;

    mutex.wait();
    vCount += q.size();
    mutex.post();


}

unsigned long int EventBottleManager::getTime()
{
    return latestStamp;

}

unsigned long int EventBottleManager::popCount()
{
    mutex.wait();
    unsigned long int r = vCount;
    vCount = 0;
    mutex.post();
    return r;

}

//empty line to make gcc happy
