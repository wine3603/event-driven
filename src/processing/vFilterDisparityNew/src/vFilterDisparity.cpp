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

#include "vFilterDisparity.h"

/**********************************************************/
bool vFilterDisparityModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vFilterDisparity")).asString();
    yarp::os::RFModule::setName(moduleName.c_str());

    bool strictness = rf.check("strict", yarp::os::Value(false)).asBool();

    /* set parameters */
    int height = rf.check("height", yarp::os::Value(128)).asInt();
    int width = rf.check("width", yarp::os::Value(128)).asInt();

    //number of events to process
    int nevents = rf.check("nevets", yarp::os::Value(100)).asInt();

//    //disparity values
//    yarp::os::Bottle disparitylist = rf.findGroup("DISPARITY");

    //maximum computable disparity
    int maxdisp = rf.check("maxdisp", yarp::os::Value(10)).asInt();

    //number of filters' directions
    int directions = rf.check("directions", yarp::os::Value(8)).asInt();

    //number of filters' phase-shifts
    int phases = rf.check("phases", yarp::os::Value(21)).asInt();

    //size of the spatial window
    int winsize = rf.check("winsize", yarp::os::Value(10)).asInt();

    bool temp_decay = rf.check("decay", yarp::os::Value('on')).asBool();

//    if(!disparitymanager->setDisparity(disparitylist)) {
//        std::cerr << "Disparity file required to run vFilterDisparity" << std::endl;
//        return false;
//    }

    /* create the thread and pass pointers to the module parameters */
    disparitymanager = new vFilterDisparityManager(height, width, nevents, directions, phases, winsize, temp_decay);
    return disparitymanager->open(moduleName, strictness);

}

/**********************************************************/
bool vFilterDisparityModule::interruptModule()
{
    disparitymanager->interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vFilterDisparityModule::close()
{
    disparitymanager->close();
    delete disparitymanager;
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vFilterDisparityModule::updateModule()
{
    return true;
}

/**********************************************************/
double vFilterDisparityModule::getPeriod()
{
    return 0.1;
}

bool vFilterDisparityModule::respond(const yarp::os::Bottle &command,
                              yarp::os::Bottle &reply)
{
    //fill in all command/response plus module update methods here
    return true;
}

/******************************************************************************/
//vFilterDisparityManager
/******************************************************************************/
vFilterDisparityManager::vFilterDisparityManager(int height, int width, int nevents, int directions, int phases, int winsize, bool temp_decay)
{
    this->height = height;
    this->width = width;
    this->nevents = nevents;
    this->directions = directions;
    this->phases = phases;
    this->winsize = winsize;
    this->temp_decay = temp_decay;

    dir_vector.resize(directions);
    //set the direction vector
    double value = 0;
    double dir_step = 0.125;
    for(int i = 0; i < directions; i++) {
        dir_vector[i] = value * M_PI; //dir_vector.push_back(value * M_PI);
        value = value + dir_step;
    }

    disparity_vector.resize(phases);
    phase_vector.resize(phases);

    //set the disparity that you want and the phase vector accordingly
    double disp_gen[] = {-10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    for(int k = 0; k < phases; k++) {
        phase_vector[k] = -disp_gen[k]*st_filters.omega;
        disparity_vector[k] = disp_gen[k];
    }

    even_conv_right.resize(phases);
    odd_conv_right.resize(phases); //Arrays of size = number of phases

    outDisparity.open("estimatedDisparityNEW.txt");
    gaborResponse.open("gaborResponseNEW.txt");

}

/**********************************************************/
bool vFilterDisparityManager::open(const std::string moduleName, bool strictness)
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
void vFilterDisparityManager::close()
{
    outDisparity.close();
    gaborResponse.close();

    //close ports
    outPort.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();

}

/**********************************************************/
void vFilterDisparityManager::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();
}

/**********************************************************/
bool vFilterDisparityManager::setDisparity(yarp::os::Bottle disparitylist)
{
    if(disparitylist.isNull())
        return false;

    this->disparitylist = disparitylist;
    return true;

}

/**********************************************************/
std::vector<int> vFilterDisparityManager::prepareDisparities(){

    std::vector<int> vDisparities;

    for(int i = 1; i < disparitylist.size(); i++)
        vDisparities.push_back(disparitylist.get(i).asList()->get(1).asInt());

    return vDisparities;

}

/**********************************************************/
void vFilterDisparityManager::onRead(emorph::vBottle &bot)
{
    /*prepare output vBottle with AEs extended with optical flow events*/
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle.clear();
    yarp::os::Stamp st;
    this->getEnvelope(st); outPort.setEnvelope(st);

    emorph::DisparityEvent *de = NULL;

    /*get the event queue in the vBottle bot*/
    emorph::vQueue q = bot.get<emorph::AddressEvent>();

    //processing queue of events
    emorph::vQueue procQueue;

    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(!aep) continue;

        procQueue.push_back(aep);

        bool removed = false;

        //ADD THE CURRENT EVENT
        FIFO.push_front(*qi);

        //KEEP FIFO TO LIMITED SIZE
        while(FIFO.size() > nevents) {
            procQueue.push_back(FIFO.back());
            FIFO.pop_back();
            removed = true;
        }

//        //update event buffer with information from both left and right eye
//        event_history.updateList(*aep);

        //current event
        x = aep->getX();
        y = aep->getY();
        ts = unwrapper(aep->getStamp()); // / 1000000;

        if(aep->getChannel() == 0) {//left channel used as reference for disparity

            //center the filters on the current event
            st_filters.center_x = aep->getX();
            st_filters.center_y = aep->getY();

            double disparityX = 0; double disparityY = 0;

            //process for each direction
            for(std::vector<double>::iterator it = dir_vector.begin(); it != dir_vector.end(); it ++) {

                theta = *it;

                //convolve the events in the buffer with Gabor filters
                //this will give you even and odd components of the response for right and left events in the history buffer
                convolveGabor();

                //estimate the disparity for the single direction
                double disparity = estimateDisparity();

                disparityX = disparityX + disparity * cos(theta);
                disparityY = disparityY + disparity * sin(theta);

            }

            disparityX = (2.0/directions)*disparityX;
            disparityY = (2.0/directions)*disparityY;

            outDisparity << ts << " " << disparityX << " " << disparityY << "\n";
            std::cout << "dx = " << disparityY << " dy = " << disparityX << std::endl;

            de = new emorph::DisparityEvent(*aep);
            de->setDx(disparityY);
            de->setDy(disparityX);

//            std::cout << "send dx = " << de->getDx() << " send dy = " << de->getDy() << std::endl;
            outBottle.addEvent(*de);

        }

    if (strictness) outPort.writeStrict();
    else outPort.write();

    }

}

/**********************************************************/
void vFilterDisparityManager::convolveGabor(){

    //reset filter convolution value for every event processing
    even_conv_left = 0; odd_conv_left = 0;
    std::fill(even_conv_right.begin(), even_conv_right.end(), 0.0);
    std::fill(odd_conv_right.begin(), odd_conv_right.end(), 0.0);

    int eventsinthewindow = 0;
    //for all the events in the list
    for(emorph::vQueue::iterator fi = FIFO.begin(); fi != FIFO.end(); fi++)
    {

//        std::cout << "size " << FIFO.size() << std::endl;

        //        //do the computation for all the events inside the spatial window
        //        for(int j = 1; j < winsize; j++) {
        //            for(int i = 1; i < winsize; i++) {
        //                int pixel_x = x + i - ((winsize + 1) / 2);
        //                int pixel_y = y + j - ((winsize + 1) / 2);

//        //check for borders
//        if(pixel_x < 0 && pixel_y < 0 && pixel_x >= height && pixel_y >= width)
//            continue;

        //                //going from latest event pushed in the back
        //                event_history.timeStampsList_it = event_history.timeStampList[pixel_x][pixel_y].rbegin();

        //                //if the list is empty skip computing
        //                if(event_history.timeStampList[pixel_x][pixel_y].empty()) continue;

        //do the computation for all the events in the temporal list
        //                for(int list_length = 1; list_length <= event_history.timeStampList[pixel_x][pixel_y].size() ;) {

        emorph::AddressEvent *vp = (*fi)->getAs<emorph::AddressEvent>();

        //check for borders
        if(vp->getX() < 0 && vp->getY() < 0 && vp->getX() >= height && vp->getY() >= width)
            continue;

        //if the event is in the spatial window
        if(abs(x - vp->getX()) < winsize && abs(y - vp->getY()) < winsize)
        {
            eventsinthewindow++;
            int dx = vp->getX() - x;
            int dy = vp->getY() - y;
            int dt = vp->getStamp() - ts;
            int ch = vp->getChannel();

            //                    int dx = pixel_x - x;
            //                    int dy = pixel_y - y;
            //                    double dt = ts - abs(*event_history.timeStampsList_it);
            //                    int ch = (*event_history.timeStampsList_it) / abs(*event_history.timeStampsList_it);
            //                    std::cout << "ts " << ts << std::endl;
            //                    std::cout << "event history ts " << abs(*event_history.timeStampsList_it) << std::endl;


            //                    if(dt > dt_step){
            //                        //std::cout << "Skipping the list element..." << std::endl;//Debug Code
            //                        ++list_length;
            //                        continue;
            //                    }

            if(ch == 0){ //(ch == -1){ //left events are encoded with -1 in the history buffer (reference channel)

                double psi = 0.0;
                std::pair<double,double> conv_value = st_filters.filtering(dx, dy, theta, dt, psi, temp_decay);
                even_conv_left = even_conv_left + conv_value.first;
                odd_conv_left  = odd_conv_left  + conv_value.second;
//                if(eventsinthewindow > 100) {
//                    std::cout << "events " << eventsinthewindow << std::endl;
//                    std::cout << "mono even left " << even_conv_left << std::endl;
//                }

            }
            else {
           // if(ch == 1){ //right events

                double psi = 0;
                std::vector<double>::iterator it = phase_vector.begin();
                for(int t = 0; t < phases; t++){

                    psi = *it;
                    std::pair<double,double> conv_value = st_filters.filtering(dx, dy, theta, dt, psi, temp_decay);
                    even_conv_right[t] = even_conv_right[t] + conv_value.first;
                    odd_conv_right[t]  = odd_conv_right[t]  + conv_value.second;

                    ++it;

                }
            }

            //                    ++event_history.timeStampsList_it; //moving up the list
            //                    ++list_length;

            //                } //end of temporal iteration loop

        }
    }

//    std::cout << "evts in the window " << eventsinthewindow << std::endl;

//    std::cout << std::endl;

}

 /**********************************************************/
double vFilterDisparityManager::estimateDisparity(){

    double final_even_conv = 0; double final_odd_conv = 0;
    double energy = 0; double energy_sum = 0;
    double disparity_sum = 0;

    std::vector<double>::iterator disparity_it = disparity_vector.begin();
    for(int t = 0; t < phases; t++){

        final_even_conv = even_conv_left + even_conv_right[t];
        final_odd_conv  = odd_conv_left  + odd_conv_right[t];
        energy = final_even_conv * final_even_conv + final_odd_conv * final_odd_conv;
        energy_sum = energy_sum + energy;
        disparity_sum = disparity_sum + (*disparity_it) * energy;

        gaborResponse << ts << " " << theta << " " << *disparity_it << " " << energy << "\n";
//        std::cout << "disparity ( " << theta * (180 / M_PI) << " ) = " << *disparity_it << " with energy = " << energy << "\n";

        ++disparity_it;

    }

    return(disparity_sum / energy_sum);

}

//empty line to make gcc happy
