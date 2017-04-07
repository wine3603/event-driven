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

bool saccadeModule::configure(yarp::os::ResourceFinder &rf) {
    
    //set the name of the module
    std::string moduleName = rf.check("name", yarp::os::Value("autoSaccade")).asString();
    setName(moduleName.c_str());
    
    //open and attach the rpc port
    std::string rpcPortName  =  "/" + moduleName + "/rpc:i";
    
    if (!rpcPort.open(rpcPortName))
    {
        std::cerr << getName() << " : Unable to open rpc port at " << rpcPortName << std::endl;
        return false;
    }
    attach(rpcPort);
    
    //open driver for joint control
    yarp::os::Property options;
    options.put("device","remote_controlboard");
    options.put("remote","/icubSim/head");
    options.put("local", "/" + moduleName + "/head");
    
    mdriver.open(options);
    if(!mdriver.isValid())
        std::cerr << "Did not connect to robot/simulator" << std::endl;
    else {
        mdriver.view(ilim);
        mdriver.view(ipos);
        mdriver.view(imod);
    }
    configDriver();
    
    //open driver for gaze control
    options.clear();
    options.put("device", "gazecontrollerclient");
    options.put("local", "/" + moduleName + "/gazeCtrl");
    options.put("remote","/iKinGazeCtrl");
    gazeDriver.open(options);
    if(!gazeDriver.isValid())
        std::cerr << "Did not connect to robot/simulator" << std::endl;
    else {
        gazeDriver.view(gazeControl);
    }
    
    if(!gazeControl)
        std::cerr << "Did not connect to gaze controller" << std::endl;
    
    gazeControl->storeContext(&context0);
    
    //initialize variables 
    isSaccading = false;
    
    //Read parameters
    checkPeriod = rf.check("checkPeriod", yarp::os::Value(0.1)).asDouble();
    minVpS = rf.check("minVpS", yarp::os::Value(20000)).asDouble();
    saccadeTimeout = rf.check("timeout", yarp::os::Value(1.0)).asDouble();
    prevStamp =  0; //max value
    
    //opening ports
    eventBottleManager.open("/" + moduleName + "/vBottle:i");
    vRate.open("/" + moduleName + "/vRate:o");
    
    return true ;
}

void saccadeModule::configDriver() {
    if ( ilim && ipos && imod ) {
        ilim->getLimits( 3, &min, &max );
        ilim->getLimits( 4, &min, &max );
        ipos->setRefSpeed( 3, 300.0 );
        ipos->setRefSpeed( 4, 300.0 );
        ipos->setRefAcceleration( 3, 200.0 );
        ipos->setRefAcceleration( 4, 200.0 );
        imod->setControlMode( 3, VOCAB_CM_POSITION );
        imod->setControlMode( 4, VOCAB_CM_POSITION );
    } else {
        std::cerr << "Could not open driver" << std::endl;
    }
}

bool saccadeModule::interruptModule() {
    std::cout << "Interrupting" << std::endl;
    rpcPort.interrupt();
    eventBottleManager.interrupt();
    std::cout << "Finished Interrupting" << std::endl;
    return true;
}

bool saccadeModule::close() {
    
    std::cout << "Closing" << std::endl;
    rpcPort.close();
    eventBottleManager.close();
    mdriver.close();
    gazeDriver.close();
    delete ilim; delete ipos; delete imod;
    std::cout << "Finished Closing" << std::endl;
    return true;
}

void saccadeModule::performSaccade() {
    for ( double theta = 0; theta < 2*M_PI; theta+= M_PI/36 ) {
        ipos->positionMove( 3, cos( theta ) );
        ipos->positionMove( 4, sin( theta ) );
        yarp::os::Time::delay(0.005);
    }
}

bool saccadeModule::updateModule() {
    
    eventBottleManager.start();
    yarp::os::Time::delay(1.0);
    eventBottleManager.stop();
    
    //if there is no connection don't do anything yet
    if(!eventBottleManager.getInputCount()) return true;
    
    //compute event rate
    double latestStamp = eventBottleManager.getTime();
    double vCount = eventBottleManager.popCount();
    double vPeriod = latestStamp - prevStamp;
    //this should only occur on first bottle to initialise
    if(vPeriod < 0) return true;
    vPeriod *= 80 *10e-9;
    const double eventRate = vCount / vPeriod;
    prevStamp = latestStamp;
    
    //output the event rate for debug purposes
    yarp::os::Bottle vRateBottle;
    vRateBottle.addDouble( eventRate );
    vRate.write(vRateBottle);
    
    //if event rate is low then saccade, else gaze to center of mass of events
    if(vPeriod == 0 || eventRate < minVpS) {
        std::cout << "perform saccade " <<  latestStamp << std::endl;
        gazeControl->stopControl();
        configDriver();
        performSaccade();
        yarp::os::Time::delay(saccadeTimeout);
    } else {
        yarp::sig::Vector cm = computeCenterMass();
        std::cout << "gazing" << std::endl;
        gazeControl->restoreContext(context0);
        gazeControl->lookAtMonoPixel(0,cm);
        gazeControl->waitMotionDone();
    }
    
    return true;
}

yarp::sig::Vector saccadeModule::computeCenterMass() const {
    int xCM = 0, yCM = 0;
    ev::vQueue q = eventBottleManager.getEvents();
    for ( ev::vQueue::iterator i = q.begin(); i != q.end(); ++i ) {
            auto aep = ev::is_event<ev::AE>( *i );
            xCM += aep.get()->x;
            yCM += aep.get()->y;
        }
    xCM /= q.size();
    yCM /= q.size();
    yarp::sig::Vector cm( 2 );
    cm(0)= xCM;
    cm(1)= yCM;
    return cm;
}

double saccadeModule::getPeriod() {
    return checkPeriod;
}

bool saccadeModule::respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply) {
    //fill in all command/response plus module update methods here
    return true;
}

/****************EventBottleManager**********************/

EventBottleManager::EventBottleManager() {
    
    //here we should initialise the module
    vCount = 0;
    latestStamp = 0;
    isReading = false;
}

bool EventBottleManager::open(const std::string &name) {
    //and open the input port
    
    this->useCallback();
    
    yarp::os::BufferedPort<ev::vBottle>::open(name);
    return true;
}

void EventBottleManager::onRead(ev::vBottle &bot) {
    if (!isReading)
        return;
    //create event queue
    vQueue = bot.get<ev::AE>();
    
    // get the event queue in the vBottle bot
    if(vQueue.empty()) return;
    
    latestStamp = unwrapper(vQueue.back()->stamp);
    
    
    mutex.wait();
    vCount += vQueue.size();
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

bool EventBottleManager::start() {
    isReading = true;
    return true;
}

bool EventBottleManager::stop() {
    isReading = false;
    return true;
}

ev::vQueue EventBottleManager::getEvents() const{
    return vQueue;
}

//empty line to make gcc happy
