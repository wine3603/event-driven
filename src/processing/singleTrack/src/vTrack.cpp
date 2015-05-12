/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Chiara Bartolozzi
 * email:  chiara.bartolozzi@iit.it
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

#include "vTrack.h"

/**********************************************************/
bool vTrackModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vTrack")).asString();
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


    //set other variables we need from the
    std::string fileName = rf.check("variable",
                        yarp::os::Value("variable_defualt"),
                        "variable description").asString();
    
    int thrNumAE = rf.check("thrNumAE", yarp::os::Value(10)).asInt();
    //eventBottleManager.setThrNumAE(thrNumAE);

    /* create the thread and pass pointers to the module parameters */
    eventBottleManager.open(moduleName);

    return true ;
}

/**********************************************************/
bool vTrackModule::interruptModule()
{
    rpcPort.interrupt();
    eventBottleManager.interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vTrackModule::close()
{
    rpcPort.close();
    
    eventBottleManager.close();
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vTrackModule::updateModule()
{
    return true;
}

/**********************************************************/
double vTrackModule::getPeriod()
{
    return 0.1;
}

/**********************************************************/
bool vTrackModule::respond(const yarp::os::Bottle &command,
                              yarp::os::Bottle &reply)
{
    //fill in all command/response plus module update methods here
    //write on the rpc port the cluster that shall be locked
    
    if (command.get(0).asString()=="lock" || command.get(0).asString()=="unlock" )
    {
        eventBottleManager.lockTracker(command, reply);
        return true;
    }
    else
    {
        printf("Unidentified command\n");
        reply.addString("wrong command");
        return false;
    }
    
}


/**********************************************************/
EventBottleManager::EventBottleManager()
{

    //here we should initialise the module
    
}
/**********************************************************/
//void EventBottleManager::setThrNumAE(int tNumAE)
//{
//    this->thrNumAE = tNumAE;
//}
/**********************************************************/
void EventBottleManager::lockTracker(const yarp::os::Bottle &command,
                                     yarp::os::Bottle &reply)
{
    int ch;
    int trackId;
    
    reply.clear();

    ch = command.get(1).asInt();
    trackId = command.get(2).asInt();
    printf("%s cluster %d, channel %d \n", command.get(0).asString().c_str(), trackId, ch);
    yarp::os::Bottle cmd;
    cmd = command;
    //cmd.addString(command.get(0).asString());
    //cmd.addInt(ch);
    //cmd.addInt(trackId);
    reply.addString(command.get(0).asString());
    rpcPortClient.write(cmd,reply);
    //rpcPortClient.write(command,reply);
}
/**********************************************************/
bool EventBottleManager::open(const std::string &name)
{
    //and open the input port

    this->useCallback();

    std::string inPortName = "/" + name + "/vBottle:i";
    yarp::os::BufferedPort<emorph::vBottle>::open(inPortName);

    std::string outPortName = "/" + name + "/vBottle:o";
    outPort.open(outPortName);
    
    std::string rpcPortClientName = "/" + name + "/rpc:o";
    rpcPortClient.open(rpcPortClientName);
    
    response.addString("unlocked");
    return true;
}

/**********************************************************/
void EventBottleManager::close()
{
    //close ports
    outPort.close();
    rpcPortClient.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();

    //remember to also deallocate any memory allocated by this class


}

/**********************************************************/
void EventBottleManager::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    rpcPortClient.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();

}

/**********************************************************/
void EventBottleManager::onRead(emorph::vBottle &bot)
{
    //create event queue
    emorph::vQueue q;
    //create queue iterator
    emorph::vQueue::iterator qi;
    
    // prepare output vBottle with address events extended with cluster ID (aec) and cluster events (clep)
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle = bot; //start with the incoming bottle

    // get the event queue in the vBottle bot
    bot.getAll(q);

    int maxNumAE = 0;
    int trackId, channel;
    
    for(qi = q.begin(); qi != q.end(); qi++)
    {
        //unwrap timestamp
        //unsigned long int ts = unwrapper((*qi)->getStamp());

        //perhaps you need to filter for a certain type of event?
        emorph::ClusterEventGauss *vCle = (*qi)->getAs<emorph::ClusterEventGauss>();
        if(!vCle) continue;
        
        if (response.toString() != "locked")
        {
            //process
            //define the conditions under which we want to lock on a cluster
            
            int numAE = vCle->getNumAE(); // get activity of each cluster and find the one with maximum activity
            
            if(numAE > maxNumAE)
            {
                maxNumAE = numAE;
                trackId = vCle->getID();
                channel = vCle->getChannel();
            
            
                if (maxNumAE > thrNumAE)
                {
                    //write on the rpc port the cluster that shall be locked
                    
                    yarp::os::Bottle cmd;
                    cmd.addString("lock");
                    cmd.addInt(channel);
                    cmd.addInt(trackId);
                    
                    {
                        printf("Sending message... %s\n", cmd.toString().c_str());
                    }
                    rpcPortClient.write(cmd,response); // or lockTracker(cmd, response);

                    {
                        printf("Got response: %s\n", response.toString().c_str());
                    }
                }
            }
        //add events that need to be added to the out bottle
        //outBottle.addEvent(**qi);
        }
    }
    //send on the processed events
    //outPort.write();


}

//empty line to make gcc happy
