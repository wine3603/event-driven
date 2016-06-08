/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Valentina Vasco and Yeshasvi Tirupachuri
 * email:  valentina.vasco@iit.it, Yeshasvi.Tirupachuri@iit.it
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
    int nevents = rf.check("nevets", yarp::os::Value(2000)).asInt();

    //number of filters' orientations
    int orientations = rf.check("orientations", yarp::os::Value(8)).asInt();

    //size of the spatial window
    int winsize = rf.check("winsize", yarp::os::Value(18)).asInt();

    //threshold to apply to the binocular energies
    double threshold = rf.check("thresh", yarp::os::Value(0)).asDouble();

    //load disparity values from configuration file
    yarp::os::Bottle disparitylist = rf.findGroup("disparity").tail();

    /* create the thread and pass pointers to the module parameters */
    disparitymanager = new vFilterDisparityManager(height, width, nevents, orientations,
                                                   winsize, threshold, disparitylist);

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
vFilterDisparityManager::vFilterDisparityManager(int height, int width, int nevents, int orientations,
                                                 int winsize, double threshold, yarp::os::Bottle disparitylist)
{
    this->height = height;
    this->width = width;
    this->nevents = nevents;
    this->orientations = orientations;
    this->winsize = winsize;
    this->threshold = threshold;
    this->disparitylist = disparitylist;
    this->phases = disparitylist.size();

    std::cout << "Setting filter parameters... " << std::endl;
    std::cout << "Number of orientations = " << orientations << std::endl;
    ori_vector.resize(orientations);
    //set the direction vector
    std::cout << "Tuned orientations = ";
    double value = 0;
    double dir_step = 1.0/orientations;
    for(int i = 0; i < orientations; i++) {
        ori_vector[i] = value * M_PI;
        value = value + dir_step;
        std::cout << ori_vector[i]*(180/M_PI) << " ";
    }
    std::cout << " degrees" << endl;

    int maxdisp = disparitylist.get(phases - 1).asInt();
    //set filters parameters
    double f_spatial = 1.0/(2 * maxdisp);
    double var_spatial = maxdisp;
    double f_temporal = 0;
    double var_temporal = 0;
    std::cout << "Spatial parameters: frequency = " << f_spatial << " standard deviation = " << var_spatial << " px" << std::endl;
    std::cout << "Temporal parameters: frequency = " << f_temporal << " standard deviation = "<< var_temporal <<std::endl;
    st_filters.setParams(f_spatial, var_spatial, f_temporal, var_temporal);

    std::cout << "Maximum disparity computable = " << maxdisp << " px " << std::endl;

    std::cout << "Number of phase-shifts = " << phases << std::endl;
    //set the disparity and the phase vector accordingly
    phase_vector.resize(phases);
    disparity_vector = NULL;
    disparity_vector = new int[phases];
    std::cout << "Tuned disparities = ";
    for(int k = 0; k < phases; k++) {
        disparity_vector[k] = disparitylist.get(k).asInt();
        phase_vector[k] = -disparity_vector[k]*(2*M_PI*st_filters.f_spatial);
        std::cout << disparity_vector[k] << " ";
    }
    std::cout << std::endl;

    even_conv_right.resize(phases);
    odd_conv_right.resize(phases);

    outDisparity.open("estimatedDisparity.txt");
    gaborResponse.open("gaborResponse.txt");

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

    delete [] disparity_vector;

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

    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(!aep) continue;

        bool removed = false;

        //ADD THE CURRENT EVENT
        FIFO.push_front(*qi);

        //KEEP FIFO TO LIMITED SIZE
        while(FIFO.size() > nevents) {
            FIFO.pop_back();
            removed = true;
        }

        //current event
        x = aep->getX();
        y = aep->getY();
        ts = unwrapper(aep->getStamp());

        if(aep->getChannel() == 0) {//left channel used as reference for disparity

            double disparityX = 0; double disparityY = 0;

            //number of orientations on which we apply the intersection of constraint
            int ori_ioc = 0;

            //process for each direction
            for(std::vector<double>::iterator it = ori_vector.begin(); it != ori_vector.end(); it ++) {

                double theta = *it;

                //compute even and odd components of the response for right and left events
                computeMonocularEnergy(theta);

                //compute binocular energy
                double disparity = computeBinocularEnergy();

                if(disparity != 0)
                    ori_ioc++;

                //estimate the disparity for the single direction
                disparityX = disparityX + disparity * cos(theta);
                disparityY = disparityY + disparity * sin(theta);

            }

            if(ori_ioc != 0) {
                disparityX = (1.0/ori_ioc)*disparityX;
                disparityY = (1.0/ori_ioc)*disparityY;
            }
//            disparityX = (2.0/orientations)*disparityX;
//            disparityY = (2.0/orientations)*disparityY;

            outDisparity << x << " " << y << " " << ts << " " << disparityX << " " << disparityY <<
                            " " << sqrt(disparityX*disparityX + disparityY*disparityY) << "\n";

//            std::cout << "disparity = " << sqrt(disparityX * disparityX + disparityY * disparityY) << std::endl;

            de = new emorph::DisparityEvent(*aep);
            de->setDx(disparityX);
            de->setDy(disparityY);

//            std::cout << "send disparity = " << sqrt(pow(de->getDx(), 2) + pow(de->getDy(), 2)) << std::endl;
            outBottle.addEvent(*de);

        }

    }

    if (strictness) outPort.writeStrict();
    else outPort.write();

}

/**********************************************************/
void vFilterDisparityManager::computeMonocularEnergy(double theta){

    //reset filter convolution value for every event processing
    even_conv_left = 0; odd_conv_left = 0;
    std::fill(even_conv_right.begin(), even_conv_right.end(), 0.0);
    std::fill(odd_conv_right.begin(), odd_conv_right.end(), 0.0);

    //for all the events in the list
    for(emorph::vQueue::iterator fi = FIFO.begin(); fi != FIFO.end(); fi++)
    {

        emorph::AddressEvent *vp = (*fi)->getAs<emorph::AddressEvent>();

        //check for borders
        if(vp->getX() < 0 && vp->getY() < 0 && vp->getX() >= height && vp->getY() >= width)
            continue;

        //if the event is in the spatial window
        if(abs(x - vp->getX()) < winsize && abs(y - vp->getY()) < winsize)
        {
            int dx = vp->getX() - x;
            int dy = vp->getY() - y;
            int dt = vp->getStamp() - ts;
            int ch = vp->getChannel();

            //compute monocular energy
            if(ch == 0){ //left events (reference channel)

                double psi = 0.0;
                std::pair<double,double> conv_value = st_filters.filtering(dx, dy, theta, dt, psi);
                even_conv_left = even_conv_left + conv_value.first;
                odd_conv_left  = odd_conv_left  + conv_value.second;
            }
            else { //right events

                double psi = 0;
                std::vector<double>::iterator it = phase_vector.begin();
                for(int t = 0; t < phases; t++){

                    psi = *it;
                    std::pair<double,double> conv_value = st_filters.filtering(dx, dy, theta, dt, psi);
                    even_conv_right[t] = even_conv_right[t] + conv_value.first;
                    odd_conv_right[t]  = odd_conv_right[t]  + conv_value.second;

                    ++it;

                }
            }

        }
    }

}

 /**********************************************************/
double vFilterDisparityManager::computeBinocularEnergy(){

    double final_even_conv = 0; double final_odd_conv = 0;
    double binocularenergy = 0; double energy_sum = 0;
    double disparity_sum = 0;

//    std::vector<int>::iterator disparity_it = disparity_vector.begin();
    for(int t = 0; t < phases; t++){

        final_even_conv = even_conv_left + even_conv_right[t];
        final_odd_conv  = odd_conv_left  + odd_conv_right[t];
        binocularenergy = final_even_conv * final_even_conv + final_odd_conv * final_odd_conv;

//        std::cout << "binocular energy " << binocularenergy << std::endl;

        //apply threshold to binocular energy
        if(binocularenergy < threshold) continue;
//            binocularenergy = 0;

        energy_sum = energy_sum + binocularenergy;
        disparity_sum = disparity_sum + disparity_vector[t] * binocularenergy;

        gaborResponse << ts << " " << " " << disparity_vector[t] << " " << binocularenergy << "\n";
//        std::cout << "disparity ( " << theta * (180 / M_PI) << " ) = " << *disparity_it << " with energy = " << energy << "\n";

    }

    if(energy_sum == 0)
        return 0;
    else
        return(disparity_sum / energy_sum);

}

//empty line to make gcc happy
