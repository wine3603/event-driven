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
    options.put("device","remote_controlboard");
    options.put("remote","/icubSim/head");
    options.put("local","/head");
    
    
    mdriver.open(options);
    if(!mdriver.isValid())
        std::cerr << "Did not connect to robot/simulator" << std::endl;
    else {
        mdriver.view(ilim);
        mdriver.view(ipos);
        mdriver.view(imod);
    }
    
    if (ilim && ipos && imod) {
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
    
    theta = 0;
    isSaccading = false;
    
    //set other variables we need from the
    checkPeriod = rf.check("checkPeriod",
                           yarp::os::Value(0.005)).asDouble();
    minVpS = rf.check("minVpS", yarp::os::Value(5000)).asDouble();
    maxVps = rf.check("maxVpS", yarp::os::Value(55000)).asDouble();
    prevStamp =  std::numeric_limits<double>::max(); //max value
    
    eventCount.open("/" + moduleName + "/eventCount:i");
    
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
    mdriver.close();
    delete ilim; delete ipos; delete imod;
    std::cout << "Finished Closing" << std::endl;
    return true;
}

void saccadeModule::performSaccade() {
    
    ipos->positionMove(3,cos(theta));
    ipos->positionMove(4,sin(theta));
    
    theta += M_PI/36;
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
    if(isSaccading){
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
    ev::vQueue q = bot.get<ev::AE>();
    
    // get the event queue in the vBottle bot
    //bot.getSorted<eventdriven::AddressEvent>(q);
    if(q.empty()) return;
    
    latestStamp = unwrapper(q.back()->stamp);
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
