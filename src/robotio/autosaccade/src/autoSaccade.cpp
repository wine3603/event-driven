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
#include <limits>

/**
 * Generates a set of fixation points to be placed in front of iCub eyes
 */
void saccadeModule::generateTrajectory(){
    double cx = 0, cy = 0.0, r = 0.15;
    double step = M_PI/18;
    for (double i = 0; i < 2*M_PI; i+= step){
        yarp::sig::Vector circlePoint(2);
        circlePoint[0] = cx + r * cos(i);
        circlePoint[1] = cy + r * sin(i);

        trajectory.push_back(circlePoint);
    }
    for (unsigned int i = 0; i < trajectory.size(); i++){
        std::cout << "point " << i << " = " << trajectory[i].toString().c_str() << std::endl;
    }
}

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
    options.put("remote","/iKinGazeCtrl");
    mdriver.open(options);
    if(!mdriver.isValid())
        std::cerr << "Did not connect to robot/simulator" << std::endl;
    else {
        mdriver.view(gazeControl);
    }


    if(!gazeControl)
        std::cerr << "Did not connect to gaze controller" << std::endl;

    gazeControl->storeContext(&context0);

    gazeControl->setTrackingMode(false);
    gazeControl->setStabilizationMode(false);
    gazeControl->blockNeckPitch();
    gazeControl->blockNeckRoll();
    gazeControl->blockNeckYaw();
    gazeControl->setEyesTrajTime(0.1);

    if(!gazeControl->getHeadPose(x,o)) {
        yError() << "Could not retrieve head pose";
        return false;
    }

    generateTrajectory();


    isSaccading = false;
    //set other variables we need from the
    checkPeriod = rf.check("checkPeriod",
                           yarp::os::Value(0.1)).asDouble();
    minVpS = rf.check("minVpS", yarp::os::Value(5000)).asDouble();
    maxVps = rf.check("maxVpS", yarp::os::Value(55000)).asDouble();
    prevStamp =  std::numeric_limits<double>::max(); //max value

    eventCount.open(moduleName + "/eventCount:i");

    return true ;
}

bool saccadeModule::interruptModule() {
    std::cout << "Interrupting" << std::endl;
    rpcPort.interrupt();
    eventCount.interrupt();
    std::cout << "Finished Interrupting" << std::endl;
    return true;
}

bool saccadeModule::close() {

    std::cout << "Closing" << std::endl;
    rpcPort.close();
    eventCount.close();
//    delete pc;
    delete gazeControl;
//    delete ec;
    mdriver.close();
    std::cout << "Finished Closing" << std::endl;
    return true;
}


void saccadeModule::performSaccade() {
    if(!gazeControl) {
        std::cerr << "Saccade cannot be performed" << std::endl;
        return;
    }

    //Computing transformation between root and head
    gazeControl->getHeadPose(x,o);

    rootToHead = yarp::math::axis2dcm(o);
    x.push_back(1);
    rootToHead.setCol(3,x);
    headToRoot = yarp::math::SE3inv(rootToHead);

    //Iteratively fixate next point of trajectory
    for (unsigned int i = 0; i < trajectory.size(); i++) {

        //Fixation point is generated in head frame coordinates
        yarp::sig::Vector currTrajPoint = trajectory[i];
        yarp::sig::Vector fixationPoint(4);
        fixationPoint[0] = currTrajPoint[0];
        fixationPoint[1] = currTrajPoint[1];
        fixationPoint[2] = 1.5;
        fixationPoint[3] = 1;

        //Trasforming into root coordinates
        fixationPoint *= headToRoot;

        gazeControl->lookAtFixationPoint(fixationPoint.subVector(0, 2));
        gazeControl->waitMotionDone(0.001,0.01);
    }

}

bool saccadeModule::updateModule() {
    //if there is no connection don't do anything yet
    if(!eventCount.getInputCount()) return true;

    yarp::os::Bottle vCountBottle;

    eventCount.read(vCountBottle);
    //check the last time stamp and count
    double latestStamp = yarp::os::Time::now();
    int vCount = vCountBottle.get(0).asInt();

    double vPeriod = latestStamp - prevStamp;

    prevStamp = latestStamp;

    //this should only occur on first bottle to initialise
    if(vPeriod < 0) return true;


    if(vPeriod == 0 || (vCount / vPeriod) < minVpS) {
           isSaccading = true;
        }

    if(vCount == 0 || vPeriod == 0 || (vCount / vPeriod) > maxVps) {
       isSaccading = false;
    }

    std::cout << vPeriod/1000000 << "s | " << vCount/vPeriod
              << " v/s" << std::endl;
    if(gazeControl && isSaccading){
        performSaccade();
        std::cout << "perform saccade: ";
    }
    return true;
}

double saccadeModule::getPeriod() {
    return checkPeriod;
}

bool saccadeModule::respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply){
    //fill in all command/response plus module update methods here
    return true;
}

EventBottleManager::EventBottleManager() {

    //here we should initialise the module
    vCount = 0;
    latestStamp = 0;

    
}

bool EventBottleManager::open(const std::string &name) {
    //and open the input port

    this->useCallback();

    std::string inPortName = "/" + name + "/vBottle:i";
    yarp::os::BufferedPort<ev::vBottle>::open(inPortName);

    return true;
}

/**********************************************************/
void EventBottleManager::onRead(ev::vBottle &bot)
{
    //create event queue
    ev::vQueue q = bot.getSorted<ev::AddressEvent>();
    //create queue iterator
    ev::vQueue::iterator qi;

    // get the event queue in the vBottle bot
    //bot.getSorted<eventdriven::AddressEvent>(q);
    if(q.empty()) return;

    latestStamp = unwrapper(q.back()->getStamp());
//    std::cout << "latestStamp = " << latestStamp << std::endl;
//    std::cout << "q.size() = " << q.size() << std::endl;

    mutex.wait();
    vCount += q.size();
    mutex.post();


}

unsigned long int EventBottleManager::getTime() {
    return latestStamp;

}

unsigned long int EventBottleManager::popCount() {
    mutex.wait();
    unsigned long int r = vCount;
    vCount = 0;
    mutex.post();
    return r;

}

//empty line to make gcc happy
