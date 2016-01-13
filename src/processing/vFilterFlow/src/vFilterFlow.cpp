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

#include "vFilterFlow.h"

/**********************************************************/
bool vFilterFlowModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vFilterFlow")).asString();
    yarp::os::RFModule::setName(moduleName.c_str());

    bool strictness = rf.check("strict", yarp::os::Value(false)).asBool();

    /* set parameters */
    int height = rf.check("height", yarp::os::Value(128)).asInt();
    int width = rf.check("width", yarp::os::Value(128)).asInt();
    int directions = rf.check("directions", yarp::os::Value(8)).asInt();
    int kernel_size = rf.check("filterSize", yarp::os::Value(31)).asInt();

    /* create the thread and pass pointers to the module parameters */
    flowmanager = new vFilterFlowManager(height, width, directions, kernel_size);
    return flowmanager->open(moduleName, strictness);

}

/**********************************************************/
bool vFilterFlowModule::interruptModule()
{
    flowmanager->interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vFilterFlowModule::close()
{
    flowmanager->close();
    delete flowmanager;
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vFilterFlowModule::updateModule()
{
    return true;
}

/**********************************************************/
double vFilterFlowModule::getPeriod()
{
    return 0.1;
}

bool vFilterFlowModule::respond(const yarp::os::Bottle &command,
                              yarp::os::Bottle &reply)
{
    //fill in all command/response plus module update methods here
    return true;
}

/******************************************************************************/
//vFilterFlowManager
/******************************************************************************/
vFilterFlowManager::vFilterFlowManager(int height, int width, int directions, int kernel_size)
{
    this->height = height;
    this->width = width;
    this->directions = directions;
    this->kernel_size = kernel_size;

    dir_vector.resize(directions);
    //set the direction vector
    double value = 0;
    double dir_step = 0.25;
    for(int i = 0; i < directions; i++) {
        dir_vector[i] = value * M_PI;
        value = value + dir_step;
    }

    response.open ("response.txt");

}
/**********************************************************/
bool vFilterFlowManager::open(const std::string moduleName, bool strictness)
{
    this->strictness = strictness;
    if(strictness) {
        std::cout << "Setting " << moduleName << " to strict" << std::endl;
        this->setStrict();
    }
    this->useCallback();

    std::string inPortName = "/" + moduleName + "/vBottle:i";
    bool check1 = BufferedPort<emorph::vBottle>::open(inPortName);

    if(strictness) outPort.setStrict();
    std::string outPortName = "/" + moduleName + "/vBottle:o";
    bool check2 = outPort.open(outPortName);
    return check1 && check2;

}

/**********************************************************/
void vFilterFlowManager::close()
{
    response.close();

    //close ports
    outPort.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();

}

/**********************************************************/
void vFilterFlowManager::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();
}

/**********************************************************/
void vFilterFlowManager::onRead(emorph::vBottle &bot)
{
    /*prepare output vBottle with AEs extended with optical flow events*/
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle.clear();
    yarp::os::Stamp st;
    this->getEnvelope(st); outPort.setEnvelope(st);

    emorph::FlowEvent *ofe = NULL;

    /*get the event queue in the vBottle bot*/
    emorph::vQueue q = bot.get<emorph::AddressEvent>();

    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(!aep) continue;
        if(aep->getChannel() != 1) continue;
        if(aep->getPolarity() == 0) continue;

        double vx = 0, vy = 0;

        //update event buffer with information from both left and right eye
        event_history.updateList(*aep);

        x = aep->getY();
        y = aep->getX();
        ts = unwrapper(aep->getStamp()) / event_history.time_scale;

        //center the filters on the current event
        st_filters.center_x = aep->getY();
        st_filters.center_y = aep->getX();

        //process for each direction
        for(std::vector<double>::iterator it = dir_vector.begin(); it != dir_vector.end(); it ++) {

            theta = *it;
            //std::cout << "theta = " << theta << std::endl;
            double energy = computeEnergy();
            //            std::cout << "theta = " << theta * (180 / M_PI) << " energy = " << energy << std::endl;

            //compute flow by integrating the energy for all the directions
            double vx0 = energy * cos(theta);
            double vy0 = energy * sin(theta);
            vx = vx + vx0;
            vy = vy + vy0;
            std::cout << "theta = " << theta * (180 / M_PI) << " energy = " << energy << " "
                         << " flowx = " << vx0 << " flowy = " << vy0 << std::endl;

            response << ts << " " << theta * (180 / M_PI) << " " << energy << "\n";
        }

        ofe = new emorph::FlowEvent(*aep);
        ofe->setVx(vx);
        ofe->setVy(vy);
        ofe->setDeath();
        std::cout << "flowx = " << ofe->getVx() << " flowy = " << ofe->getVy() << std::endl;

        if(ofe) outBottle.addEvent(*ofe);
        else outBottle.addEvent(*aep);

    }

    if (strictness) outPort.writeStrict();
    else outPort.write();

}

double vFilterFlowManager::computeEnergy()
{
    //Resetting filter convolution value for every event processing
    double even_conv = 0, odd_conv = 0, motion_energy = 0;
    double temporal_diff_step = 1;

    for(int j = 1; j <= kernel_size; j++) {
        for(int i = 1; i <= kernel_size; i++) {
            int pixel_x = x + i - ((kernel_size + 1) / 2);
            int pixel_y = y + j - ((kernel_size + 1) / 2);

            //checking for borders
            if(pixel_x >= 0 && pixel_y >= 0 && pixel_x < height && pixel_y < width) {

                //going from latest event pushed in the back
                event_history.timeStampsList_it = event_history.timeStampList[pixel_x][pixel_y].rbegin();

                //if the list is empty skip computing
                if(event_history.timeStampList[pixel_x][pixel_y].empty()) continue;

                //do the computation for all the events in the list
                for(int list_length = 1; list_length <= event_history.timeStampList[pixel_x][pixel_y].size(); list_length++) {

//                    std::cout << "size event buffer = " << event_history.timeStampList[pixel_x][pixel_y].size() << endl;
//                    std::cout << "history timestamp = " << *event_history.timeStampsList_it << endl;
//                    std::cout << "current timestamp = " << ts << endl;

                    int dx = pixel_x - x;
                    int dy = pixel_y - y;
                    double dt = ts - abs(*event_history.timeStampsList_it);
                    //                std::cout << "deltat = " << deltat << endl;

                    if(dt > temporal_diff_step){
                        //std::cout << "Skipping the list element..." << std::endl;//Debug Code
                        ++list_length;
                        continue;
                    }
                    std::pair<double,double> conv_value = st_filters.filtering(dx, dy, theta, dt);
                    even_conv = even_conv + conv_value.first;
                    odd_conv  = odd_conv  + conv_value.second;

                    ++event_history.timeStampsList_it; //Moving up the list
                }

            }

            else continue;

        }

    }

    //compute motion energy
    motion_energy = even_conv*even_conv + odd_conv*odd_conv;
    return motion_energy;

}

//empty line to make gcc happy
