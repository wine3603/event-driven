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
    int directions = rf.check("directions", yarp::os::Value(8)).asInt();
    int phases = rf.check("phases", yarp::os::Value(9)).asInt();
    int kernel_size = rf.check("filterSize", yarp::os::Value(31)).asInt();

    /* create the thread and pass pointers to the module parameters */
    disparitymanager = new vFilterDisparityManager(directions, phases, kernel_size);
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
vFilterDisparityManager::vFilterDisparityManager(int directions, int phases, int kernel_size)
{
    this->directions = directions;
    this->phases = phases;
    this->kernel_size = kernel_size;

    //set the direction vector
    double value = 0;
    dir_step = 0.25;
    for(int i = 1; i <= directions; i++) {
        dir_vector.push_back(value * M_PI);
        value = value + dir_step;
    }

    //set the phase vector
    double phase_gen[] = {-1,-0.5,-0.25,-0.125,0,0.125,0.25,0.5,1};
    double phase_value = 0;
    for(int k = 0; k <= phases; k++) {
        phase_value = phase_gen[k] * M_PI;
        phase_vector.push_back(phase_value);
    }

    //create the bank of filters
    //st_filters(0.0625, 6.5, 0.0625, 5);
    
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

    /*get the event queue in the vBottle bot*/
    emorph::vQueue q = bot.get<emorph::AddressEvent>();

    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(!aep) continue;
        int channel = aep->getChannel();

        //update event buffer with information from both left and right eye
        event_history.updateList(*aep);

        pos_x = aep->getY();
        pos_y = aep->getX();
//        int ts = unwrapper((*qi)->getStamp());

        //center the filters on the current event
        st_filters.center_x = aep->getY();
        st_filters.center_y = aep->getX();

        //process for each direction
        //for(std::vector<double>::iterator i = dir_vector.begin(); i != dir_vector.end(); i++){
          //  theta = *i;
            // processing();
        //}
    }

    if (strictness) outPort.writeStrict();
    else outPort.write();

}

/**********************************************************/
/* void vFilterDisparityManager::processing(){

  //std::cout << "Processing Events..." << std::endl; //Debug Code
  int current_event_channel;

  //Resetting filter convolution value for every event processing
  double even_conv_left = 0;
  double odd_conv_left = 0;
  double even_conv_right[phases];
  double odd_conv_right[phases]; //Arrays of size = number of phases
  double final_convolution_even[phases];
  double final_convolution_odd[phases];
  double final_convolution[phases];
  double event_time;
  double temporal_diff_step = 1;

  //Right Eye convolution variables are stored in the array of size equal to number of phases
  std::fill_n(even_conv_right, phases, 0); //Array initilization to zero
  std::fill_n(odd_conv_right, phases, 0);

  //Spatial and Temporal neighborhood processing - Left Eye
  for(int j=1 ; j<= kernel_size ; j++){
    for(int i=1 ; i <= kernel_size ; i++){

      //Pixels to be processed in the spatial neighbourhood for both eyes - SPATIAL CORRELATION
      int pixel_x = pos_x + i - 16;
      int pixel_y = pos_y + j - 16;
      double temporal_difference;

      if(pixel_x >= 0 && pixel_y>= 0 && pixel_x < MAX_RES && pixel_y < MAX_RES){  //Checking for borders

        event_history.timeStampsList_it = event_history.timeStampList[pixel_x][pixel_y].rbegin(); //Going from latest event pushed in the back

        //NOTE : List size is always limited to the buffer size, took care of this in the event buffer code
        if(!event_history.timeStampList[pixel_x][pixel_y].empty()){ //If the list is empty skip computing

          //std::cout << "Event History Buffer Size : " <<  event_history.timeStampList[pixel_x][pixel_y].size() << std::endl;//Debug Code

          for( int list_length = 1 ; list_length <= event_history.timeStampList[pixel_x][pixel_y].size() ; ){

            //NOTE : Timestamps in the list are encoded with polarity information
            temporal_difference = event_time - abs(*event_history.timeStampsList_it); //The first value is always zero
            //std::cout << "Temporal difference : " << temporal_difference <<std::endl; //Debug code

            current_event_channel = (*event_history.timeStampsList_it)/(abs(*event_history.timeStampsList_it));
            //std::cout << "Curret Event Channel : " << current_event_channel << std::endl;//Debug Code
            //Here the neighborhood processing goes over 0 - 1 time scale
            if(temporal_difference > temporal_diff_step){
              //std::cout << "Skipping the list element..." << std::endl;//Debug Code
              ++list_length;
              continue;
            }

            //NOTE: Now processing left and right channels based on the channel infomarion decoded from event history buffer
            if(current_event_channel == -1){ //Left eye events

              double phase=0.0; //Local Variable
              //Call the spatio-temporal filtering function
              std::pair<double,double> conv_value = st_filters.filtering(pixel_x, pixel_y, theta, temporal_difference,phase);

              //TODO Check the effects of Polarity here! Without polarity its fine to get just the motion energy
              even_conv_left = even_conv_left +  conv_value.first;
              odd_conv_left = odd_conv_left +   conv_value.second;

              //std::cout << "Left Eye EVEN and ODD : " << even_conv_left << " " << odd_conv_left << std::endl; //Debug Code

            }

            if(current_event_channel == 1){//Right eye events

              double phase = 0;
              std::vector<double>::iterator phase_vector_it = phase_vector.begin(); //Starting from the first
              for(int t=0; t < phases ; ){

                //std::cout << "Initial Even Right : "<< even_conv_right[t] << std::endl; //Debug Code
                //std::cout << "Initial Odd Right : "<< odd_conv_right[t] << std::endl; //Debug Code

                phase = *phase_vector_it; //Local Varialbe
                //std::cout << "Phase Value : "<<phase<<std::endl;//Debug Code
                //Call the spatio-temporal filtering function
                std::pair<double,double> conv_value = st_filters.filtering(pixel_x, pixel_y, theta, temporal_difference,phase);

                //TODO Check the effects of Polarity here! Without polarity its fine to get just the motion energy
                even_conv_right[t] = even_conv_right[t] +  conv_value.first;
                odd_conv_right[t] = odd_conv_right[t] +   conv_value.second;

                //std::cout << "Right Eye EVEN and ODD : " << even_conv_right[t] << " " << odd_conv_right[t] << std::endl; //Debug Code

                ++phase_vector_it;
                ++t;
              }
            }

            ++event_history.timeStampsList_it; //Moving up the list
            ++list_length;

          }//End of temporal iteration loop
        }
        else {
          //std::cout << "Skipping empty list of Left Eye..." << std::endl; //Debug Code
          continue;
        }
      }

      else{

          //std::cout<< "Pixels Out of Bordes...."<<std::endl; //Debug Code
        continue;
      }
    }
  }//End of spatial iteration loop
  //std::cout << "Left eye convolution values : " << even_conv_left << " " << odd_conv_left << std::endl; //Debug Code

}
*/
//empty line to make gcc happy
