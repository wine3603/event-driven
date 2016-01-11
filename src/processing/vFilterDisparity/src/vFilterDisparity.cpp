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
    int directions = rf.check("directions", yarp::os::Value(8)).asInt();
    int phases = rf.check("phases", yarp::os::Value(9)).asInt();
    int kernel_size = rf.check("filterSize", yarp::os::Value(31)).asInt();

    /* create the thread and pass pointers to the module parameters */
    disparitymanager = new vFilterDisparityManager(height, width, directions, phases, kernel_size);
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
vFilterDisparityManager::vFilterDisparityManager(int height, int width, int directions, int phases, int kernel_size)
{
    this->height = height;
    this->width = width;
    this->directions = directions;
    this->phases = phases;
    this->kernel_size = kernel_size;

    dir_vector.resize(directions);
    //set the direction vector
    double value = 0;
    dir_step = 0.125;
    for(int i = 0; i < directions; i++) {
        dir_vector[i] = value * M_PI; //dir_vector.push_back(value * M_PI);
        value = value + dir_step;
    }

    disparity_vector.resize(phases);
    phase_vector.resize(phases);
    binocular_energy_theta.resize(phases);

    if(phases == 1) {
        phase_vector[0] = 0;
        disparity_vector[0] = phase_vector[0] / st_filters.omega;
    }
    else {
        //set the phase vector and the disparity vector
        double phase_gen[] = {-1,-0.5,-0.25,-0.125,0,0.125,0.25,0.5,1};
        double phase_value = 0;
        for(int k = 0; k < phases; k++) {
            phase_value = phase_gen[k] * M_PI;
            phase_vector[k] = phase_value;
            disparity_vector[k] = phase_value / st_filters.omega;
            //phase_vector.push_back(phase_value);
            //disparity_vector.push_back(phase_value / st_filters.omega);
        }
    }

    even_conv_right.resize(phases);
    odd_conv_right.resize(phases); //Arrays of size = number of phases
    final_convolution_even.resize(phases);
    final_convolution_odd.resize(phases);
    final_convolution.resize(phases);

    disparity_est_theta.resize(directions);

    gaborResponse.open ("gaborResponse.txt");

    //create the bank of filters
    //st_filters; //(0.0625, 6.5, 0.0625, 5);

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
void vFilterDisparityManager::onRead(emorph::vBottle &bot)
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
        int channel = aep->getChannel();

        //update event buffer with information from both left and right eye
        event_history.updateList(*aep);

        pos_x = aep->getY();
        pos_y = aep->getX();
        ts = unwrapper(aep->getStamp()) / event_history.time_scale;
//        std::cout << ts << std::endl;

        //center the filters on the current event
        st_filters.center_x = aep->getY();
        st_filters.center_y = aep->getX();

        int thetai = 0;
        if(channel == 0) {
            //process for each direction
            std::vector<double>::iterator dir_vector_it = dir_vector.begin();
            for(; dir_vector_it != dir_vector.end() ;) {
                theta = *dir_vector_it;

                //convolve the events in the buffer with Gabor filters
                convolveGabor();

                //compute binocular energy, which will be a vector of energies,
                //each corresponding to each phase
                binocular_energy_theta = computeEnergy();
                //std::cout << "energy for direction " << theta << " = " << binocular_energy_theta[2] << std::endl;

                //compute disparity for all the phases for a single direction
                double disparityTheta = computeDisparityTheta();
//                std::cout << "disparity for direction " << theta << " = " << disparityTheta << std::endl;

                disparity_est_theta[thetai] = disparityTheta; //disparity_est_theta.push_back(disparityTheta);
//                std::cout << "disparity for direction " << thetai << " = " << disparity_est_theta[thetai] << std::endl;

                //clearing the binocular energy vector of each direction
                binocular_energy_theta.clear();

                ++dir_vector_it;
                ++thetai;

            }

            //compute flow
            ofe = computeFlow(*aep);
//            std::cout << "vx " << ofe->getVx() << " vy " << ofe->getVy() << std::endl;
            if(ofe) outBottle.addEvent(*ofe);
            else outBottle.addEvent(*aep);

            //estimate the final disparity for all the directions
            std::pair<double,double> disparity = estimateDisparity();
//            std::cout << "Dx " << disparity.first << " Dy " << disparity.second << std::endl;

            //clearing the disparity estimate vector
            disparity_est_theta.clear();

            //for disparity
            ofe = new emorph::FlowEvent(*aep);
            ofe->setVx(disparity.first);
            ofe->setVx(disparity.second);
            outBottle.addEvent(*ofe);

        }

    if (strictness) outPort.writeStrict();
    else outPort.write();

    }

}

/**********************************************************/
 void vFilterDisparityManager::convolveGabor(){

  //std::cout << "Processing Events..." << std::endl; //Debug Code
  int current_event_channel;
  double temporal_diff_step = 1;

  //Resetting filter convolution value for every event processing
  even_conv_left = 0;
  odd_conv_left = 0;
  std::fill(even_conv_right.begin(), even_conv_right.end(), 0.0);
  std::fill(odd_conv_right.begin(), odd_conv_right.end(), 0.0);

  //Spatial and Temporal neighborhood processing - Left Eye
  for(int j=1 ; j <= kernel_size ; j++){
    for(int i=1 ; i <= kernel_size ; i++){

      //Pixels to be processed in the spatial neighbourhood for both eyes - SPATIAL CORRELATION
      int pixel_x = pos_x + i - 16;
      int pixel_y = pos_y + j - 16;

      //Checking for borders
      if(pixel_x >= 0 && pixel_y >= 0 && pixel_x < height && pixel_y < width){

          //Going from latest event pushed in the back
          event_history.timeStampsList_it = event_history.timeStampList[pixel_x][pixel_y].rbegin();

          //NOTE : List size is always limited to the buffer size, took care of this in the event buffer code
          if(!event_history.timeStampList[pixel_x][pixel_y].empty()){ //If the list is empty skip computing

              std::cout << "Event History Buffer Size : " <<  event_history.timeStampList[pixel_x][pixel_y].size() << std::endl;//Debug Code

              for( int list_length = 1 ; list_length <= event_history.timeStampList[pixel_x][pixel_y].size() ; ){

                  //NOTE : Timestamps in the list are encoded with polarity information
                  double temporal_difference = ts - abs(*event_history.timeStampsList_it); //The first value is always zero
//                  std::cout << "Event history : " << *event_history.timeStampsList_it << std::endl; //Debug code
//                  std::cout << "Temporal difference : " << temporal_difference <<std::endl; //Debug code

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
                      even_conv_left = even_conv_left + conv_value.first;
                      odd_conv_left  = odd_conv_left  + conv_value.second;

                      //std::cout << "Left Eye EVEN and ODD : " << even_conv_left << " " << odd_conv_left << std::endl; //Debug Code

                  }

                  if(current_event_channel == 1){//Right eye events

                      double phase = 0;
                      std::vector<double>::iterator phase_vector_it = phase_vector.begin(); //Starting from the first
                      for(int t=0; t < phases ; ){

                          //std::cout << "Initial Even Right : "<< even_conv_right[t] << std::endl; //Debug Code
                          //std::cout << "Initial Odd Right : "<< odd_conv_right[t] << std::endl; //Debug Code

                          phase = *phase_vector_it; //Local Varialbe
//                          std::cout << "Phase Value : "<<phase<<std::endl;//Debug Code
                          //Call the spatio-temporal filtering function
                          std::pair<double,double> conv_value = st_filters.filtering(pixel_x, pixel_y, theta, temporal_difference,phase);

                          //TODO Check the effects of Polarity here! Without polarity its fine to get just the motion energy
                          even_conv_right[t] = even_conv_right[t] +  conv_value.first;
                          odd_conv_right[t]  = odd_conv_right[t]  +  conv_value.second;

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

/**********************************************************/
std::vector<double> vFilterDisparityManager::computeEnergy() {

    //Final Convolution values reset to zero
    std::fill(final_convolution_even.begin(), final_convolution_even.end(), 0.0);
    std::fill(final_convolution_odd.begin(), final_convolution_odd.end(), 0.0);
    std::fill(final_convolution.begin(), final_convolution.end(), 0.0);

    //Energy Response of Quadrature pair, Even and Odd filters - Simple Binocular Cell Response
    //disparity_vector_it = disparity_vector.begin();
    std::vector<double>::iterator disp_vector_it = disparity_vector.begin(); //Starting from the first
    double disp = 0;
    for(int t=0; t < phases ; ){

      //std::cout << "Computing Binocular Energy along each phase..." << std::endl;//Debug Code
      //std::cout << "Left Eye EVEN and ODD : " << even_conv_left << " " << odd_conv_left << std::endl; //Debug Code
      //std::cout << "Right Eye EVEN and ODD : " << even_conv_right[t] << " " << odd_conv_right[t] << std::endl; //Debug Code

      final_convolution_even[t]= even_conv_left + even_conv_right[t];
      final_convolution_odd[t]= odd_conv_left + odd_conv_right[t];

      //std::cout << "Final Convolution EVEN and ODD : " << final_convolution_even[t] << " " << final_convolution_odd[t] << std::endl; //Debug Code

      final_convolution[t] = (final_convolution_even[t]*final_convolution_even[t]) + (final_convolution_odd[t] * final_convolution_odd[t]);
      //std::cout << theta_index << " " << *disparity_vector_it << " " << final_convolution[t] << std::endl ; //Debug Code

      binocular_energy_theta[t] = final_convolution[t]; //binocular_energy_theta.push_back(final_convolution[t]);
      //disparity_vector_it++;

      disp = *disp_vector_it;

//      gaborResponse.precision(10);
//      gaborResponse.setf(ios::fixed);
//      gaborResponse.setf(ios::showpoint);
      gaborResponse << ts << " " << theta << " " << disp << " " << binocular_energy_theta[t] << "\n";

      ++disp_vector_it;
      ++t;
  }

    return binocular_energy_theta;

 }

/**********************************************************/
double vFilterDisparityManager::computeDisparityTheta() {

    double phase_sum = 0;
    double energy_sum = 0;
    std::vector<double>::iterator binocular_energy_theta_it = binocular_energy_theta.begin();
    std::vector<double>::iterator disparity_vector_it = disparity_vector.begin();

    for(int t=0; t < phases ; ){

      //Check the exact values accessed from the vectors
      //std::cout << "Disparity value : " << *disparity_vector_it << std::endl; //Debug Code

      energy_sum = energy_sum + *binocular_energy_theta_it;

      //std::cout << "Binocular Energy : " << *binocular_enery_theta_it << std::endl; //Debuc Code
      phase_sum = phase_sum + (*binocular_energy_theta_it * *disparity_vector_it );

      ++binocular_energy_theta_it;
      ++disparity_vector_it;
      ++t;

    }

    return (phase_sum / energy_sum);

}

/**********************************************************/
std::pair<double, double> vFilterDisparityManager::estimateDisparity(){

    //Disparity components set to zero
    double Dx = 0;
    double Dy = 0 ;

    std::vector<double>::iterator dir_vector_it = dir_vector.begin();
    std::vector<double>::iterator disparity_est_theta_it = disparity_est_theta.begin();
    //std::cout << "Size of Binocular Energy Vector : " << binocular_enery_theta.size()<<std::endl; //Debug Code
    //std::cout << "Convolution value after filtering : " << final_convolution << endl; //Debug Code

    for(; dir_vector_it != dir_vector.end() - 1 ; ){//Summing up motion estimation along all directons

      //std::cout << "Theta value : " << *theta_vector_it<< std::endl; //Debug Code

//      Dx = Dx + ( *disparity_est_theta_it   *  cos( *dir_vector_it ) );
//      Dy = Dy - ( *disparity_est_theta_it   *  sin( *dir_vector_it ) );

        //CHANGED
        Dx = Dx + ( *disparity_est_theta_it   *  cos( *dir_vector_it ) );
        Dy = Dy + ( *disparity_est_theta_it   *  sin( *dir_vector_it ) );

      //std::cout << "Velocity Components Ux : " << Dx << " Uy : " << Dy << std::endl; //Debug Code

      ++disparity_est_theta_it ;
      ++dir_vector_it;

    }

    //ADDED
    Dx = 2 * (1/directions) * Dx;
    Dy = 2 * (1/directions) * Dy;

    return std::make_pair(Dx, Dy);
}

/**********************************************************/
emorph::FlowEvent * vFilterDisparityManager::computeFlow(emorph::AddressEvent &ae){

    emorph::FlowEvent * vf = NULL;

    //Disparity components set to zero
    double vx = 0;
    double vy = 0;

    std::vector<double>::iterator dir_vector_it = dir_vector.begin();
    std::vector<double>::iterator binocular_energy_theta_it = binocular_energy_theta.begin();
    //std::cout << "Size of Binocular Energy Vector : " << binocular_enery_theta.size()<<std::endl; //Debug Code
    //std::cout << "Convolution value after filtering : " << final_convolution << endl; //Debug Code

    for(; dir_vector_it != dir_vector.end() - 1 ; ){//Summing up motion estimation along all directons

      //std::cout << "Theta value : " << *dir_vector_it<< std::endl; //Debug Code
      //std::cout << "Binocular value : " << *binocular_energy_theta_it<< std::endl; //Debug Code

      vx = vx + ( *binocular_energy_theta_it   *  cos( *dir_vector_it ) )  ;
      vy = vy - ( *binocular_energy_theta_it   *  sin( *dir_vector_it ) ) ;

      //std::cout << "Velocity Components vx : " << vx << " vy : " << vy << std::endl; //Debug Code

      ++binocular_energy_theta_it;
      ++dir_vector_it;

    }

    //std::cout << "Velocity Components vx : " << vx << " vy : " << vy << std::endl; //Debug Code

    vf = new emorph::FlowEvent(ae);
    vf->setVx(vx);
    vf->setVy(vy);

    return vf;
}

//empty line to make gcc happy
