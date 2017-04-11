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

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

bool saccadeModule::configure(ResourceFinder &rf) {
    
    //set the name of the module
    string moduleName = rf.check("name", Value("autoSaccade")).asString();
    setName(moduleName.c_str());
    
    //open and attach the rpc port
    string rpcPortName  =  "/" + moduleName + "/rpc:i";
    
    if (!rpcPort.open(rpcPortName))
    {
        cerr << getName() << " : Unable to open rpc port at " << rpcPortName << endl;
        return false;
    }
    attach(rpcPort);
    
    //open driver for joint control
    Property options;
    options.put("device","remote_controlboard");
    options.put("remote","/icubSim/head");
    options.put("local", "/" + moduleName + "/head");
    
    mdriver.open(options);
    if(!mdriver.isValid())
        cerr << "Did not connect to robot/simulator" << endl;
    else {
        mdriver.view(ilim);
        mdriver.view(ipos);
        mdriver.view(imod);
    }
    
    for ( int i = 0; i < 5; ++i ) {
        configDriver(i );
    }
    
    //open driver for gaze control
    options.clear();
    options.put("device", "gazecontrollerclient");
    options.put("local", "/" + moduleName + "/gazeCtrl");
    options.put("remote","/iKinGazeCtrl");
    gazeDriver.open(options);
    if(!gazeDriver.isValid())
        cerr << "Did not connect to robot/simulator" << endl;
    else {
        gazeDriver.view(gazeControl);
    }
    
    if(!gazeControl)
        cerr << "Did not connect to gaze controller" << endl;
    
    gazeControl->storeContext(&context0);
    
    //initialize variables 
    isSaccading = false;
    
    //Read parameters
    checkPeriod = rf.check("checkPeriod", Value(0.1)).asDouble();
    minVpS = rf.check("minVpS", Value(8000)).asDouble();
    saccadeTimeout = rf.check("timeout", Value(1.0)).asDouble();
    prevStamp =  0; //max value
    
    //opening ports
    eventBottleManager.open("/" + moduleName + "/vBottle:i");
    vRate.open("/" + moduleName + "/vRate:o");
    imgL.open("/" + moduleName + "/imgL:o");
    imgR.open("/" + moduleName + "/imgR:o");
    return true ;
}

void saccadeModule::configDriver( int joint ) {
    if ( ilim && ipos && imod ) {
        ilim->getLimits( joint, &min, &max );
        ipos->setRefSpeed( joint, 300.0 );
        ipos->setRefAcceleration( joint, 200.0 );
        imod->setControlMode( joint, VOCAB_CM_POSITION );
    } else {
        cerr << "Could not open driver" << endl;
    }
}

bool saccadeModule::interruptModule() {
    cout << "Interrupting" << endl;
    rpcPort.interrupt();
    eventBottleManager.interrupt();
    cout << "Finished Interrupting" << endl;
    return true;
}

bool saccadeModule::close() {
    
    cout << "Closing" << endl;
    rpcPort.close();
    eventBottleManager.close();
    mdriver.close();
    gazeDriver.close();
    delete ilim; delete ipos; delete imod;
    cout << "Finished Closing" << endl;
    return true;
}

void saccadeModule::performSaccade() {
    for ( double theta = 0; theta < 2*M_PI; theta+= M_PI/36 ) {
        ipos->positionMove( 3, cos( theta ) );
        ipos->positionMove( 4, sin( theta ) );
        Time::delay(0.005);
    }
}

bool saccadeModule::updateModule() {
    
    eventBottleManager.start();
    Time::delay(1.0);
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
    Bottle vRateBottle;
    vRateBottle.addDouble( eventRate );
    vRate.write(vRateBottle);
    
    ev::vQueue q = eventBottleManager.getEvents();
    vFeatureMap lMap(240,304);
    vFeatureMap rMap(240,304);
    ImageOf<PixelBgr> &leftImage = imgL.prepare();
    ImageOf<PixelBgr> &rightImage = imgR.prepare();
    for ( ev::vQueue::iterator i = q.begin(); i != q.end(); ++i ) {
        auto aep = ev::is_event<ev::AE>( *i );
        if (aep.get()->channel) {
            rMap(aep.get()->y, aep.get()->x) += 80;
        } else {
            lMap (aep.get()->y, aep.get()->x)+=80;
        }
    }
    lMap.convertToImage(leftImage);
    rMap.convertToImage(rightImage);
    
    home();
    
    //if event rate is low then saccade, else gaze to center of mass of events
    if(vPeriod == 0 || eventRate < minVpS) {
        cout << "perform saccade " <<  latestStamp << endl;
        gazeControl->stopControl();
        configDriver( 3 );
        configDriver( 4 );
        performSaccade();
        Time::delay(saccadeTimeout);
    } else {
        Vector cmL,cmR;
        computeCenterMass( cmR, cmL, q );
        cout << "gazing at l:("  << cmL(0) << ", " << cmL(1) << ")" << endl;
        cout << "          r:("  << cmR(0) << ", " << cmR(1) << ")" << endl;
        leftImage(cmL(0), cmL(1)) = PixelBgr(255,0,0);
        rightImage(cmR(0), cmR(1)) = PixelBgr(255,0,0);
        gazeControl->restoreContext(context0);
        gazeControl->lookAtStereoPixels(cmL, cmR);
        gazeControl->waitMotionDone();
    }
    
    
    imgL.write();
    imgR.write();
    return true;
}

void saccadeModule::home() {
    gazeControl->stopControl();
    
    bool motionDone;
    
    for ( int i = 0; i < 5; ++i ) {
        configDriver( i );
        ipos->positionMove( i, 0 );
        while (!motionDone){
            motionDone = true;
            motionDone &= ipos->checkMotionDone( i, &motionDone );
            Time::delay( 0.2 );
        }
    }
}

void saccadeModule::computeCenterMass( Vector &cmR, Vector &cmL, ev::vQueue &q ) {
    int xl = 0, yl = 0;
    int xr = 0, yr = 0;
    int rSize = 0, lSize = 0;
    cmR.resize(2);
    cmL.resize(2);
    for ( ev::vQueue::iterator i = q.begin(); i != q.end(); ++i ) {
        auto aep = ev::is_event<ev::AE>( *i );
        if (aep.get()->channel) {
            xr += aep.get()->x;
            yr += aep.get()->y;
            rSize++;
        } else {
            xl += aep.get()->x;
            yl += aep.get()->y;
            lSize++;
        }
    }
    
    if (lSize == 0 || rSize == 0) return;
    xl /= lSize;
    yl /= lSize;
    xr /= rSize;
    yr /= rSize;
    
    cmR(0) = xr;
    cmR(1) = yr;
    cmL(0) = xl;
    cmL(1) = yl;
    
}

double saccadeModule::getPeriod() {
    return checkPeriod;
}

bool saccadeModule::respond(const Bottle &command, Bottle &reply) {
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

bool EventBottleManager::open(const string &name) {
    //and open the input port
    
    this->useCallback();
    
    BufferedPort<ev::vBottle>::open(name);
    return true;
}

void EventBottleManager::onRead(ev::vBottle &bot) {
    if (!isReading)
        return;
    
    //get new events
    ev::vQueue newQueue = bot.get<ev::AE>();
    if(newQueue.empty()){
        return;
    }
    
    mutex.wait();
    //append new events to queue
    vQueue.insert(vQueue.end(), newQueue.begin(), newQueue.end());
    latestStamp = unwrapper(newQueue.back()->stamp);
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

ev::vQueue EventBottleManager::getEvents() {
    ev::vQueue outQueue = vQueue;
    vQueue.clear();
    return outQueue;
}

//empty line to make gcc happy
