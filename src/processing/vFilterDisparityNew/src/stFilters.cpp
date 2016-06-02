/*
 * spatialFilters.cpp
 *
 *  Created on: April 16, 2015
 *      Author: yeshasvi tvs
 */

#include "stFilters.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include "vFilterDisparity.h"
#define  BUFSIZE 256

std::pair<double,double> stFilters::filtering(int& dx,int& dy,double& theta,int& time,double& phase, bool temp_decay){ //(const emorph::vQueue& subsurf, int& curr_ts, double& theta, double& phase){ {

    double st_even = 0, st_odd = 0;

     //Spatial Filter computation
     double D = (1/ (2 * M_PI * pow(var_spatial,2))); //double D = 2 * M_PI * (1/ pow(var_spatial,2));

     //CHANGED
     double dx_theta =  dx * cos(theta) + dy * sin(theta);
     double dy_theta = -dx * sin(theta) + dy * cos(theta);
     double gaussian_spatial = exp( - ( pow(dx_theta, 2) + pow(dy_theta,2) ) / ( 2 * pow(var_spatial,2) ) ); //double gaussian_spatial = exp( - ( pow(dx,2) + pow(dy,2) ) / ( 2 * pow(var_spatial,2) ) );
     double f1 = f_spatial * dx_theta;
     // double f1 = f_spatial * ( dx * cos(-theta) + dy * sin(-theta) );

     double even_spatial = D * gaussian_spatial  * cos( (2 * M_PI * f1 ) + phase );
     double odd_spatial = D * gaussian_spatial * sin( (2 * M_PI * f1) + phase );

//     if(temp_decay) {
//         //Temporal Filter computation
//         double gaussian_temporal = exp( - pow(time,2) / (2 * pow(var_temporal,2) )  );

//         double mono_temporal = gaussian_temporal * cos( 2 * M_PI * f_temporal * time);
//         double bi_temporal =  gaussian_temporal * sin( 2 * M_PI * f_temporal * time);

//         double st1 = even_spatial * bi_temporal;
//         double st3 = odd_spatial * mono_temporal;

//         double st2 = even_spatial * mono_temporal;
//         double st4 = odd_spatial * bi_temporal;

//         //Even and Odd components of spatio-temporal filters
//         st_even = st1 + st3;
//         st_odd =  -st2 + st4; //CHECK THIS
//     }
//     else {
         st_even = even_spatial;
         st_odd = odd_spatial;
//     }

     //std::cout << "Event Convolution computed: " << st_even << " "<<st_odd<< std::endl;//Debug Code

    return std::make_pair(st_even,st_odd);

}

//Destructor definition
stFilters::~stFilters(){

}
