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

std::pair<double,double> stFilters::filtering(int& dx,int& dy,double& theta,double& time,double& phase){ //(const emorph::vQueue& subsurf, int& curr_ts, double& theta, double& phase){ {

 //    double st_even = 0, st_odd = 0;

//    for(int k = 0; k < subsurf.size(); k++) {

//        int dx = subsurf[k]->getUnsafe<emorph::AddressEvent>()->getX() - center_x;
//        int dy = subsurf[k]->getUnsafe<emorph::AddressEvent>()->getY() - center_y;
//        int time = (curr_ts - subsurf[k]->getStamp()) / 1000000;

//        //Spatial Filter computation
//        double D = 2 * M_PI * (1/ pow(var_spatial,2));
//        double gaussian_spatial = exp( - ( pow(dx, 2) + pow(dy, 2) ) / ( 2 * pow(var_spatial,2) ) );
//        double f1 = f_spatial * ( dx * cos(-theta) + dy * sin(-theta) );
//        double even_spatial = D * gaussian_spatial * cos( (2 * M_PI * f1) + phase );
//        double odd_spatial  = D * gaussian_spatial * sin( (2 * M_PI * f1) + phase );

//        //Temporal Filter computation
//        double gaussian_temporal = exp( - pow(time,2) / (2 * pow(var_temporal,2) )  );

//        double mono_temporal = gaussian_temporal * cos( 2 * M_PI * f_temporal * time);
//        double bi_temporal   = gaussian_temporal * sin( 2 * M_PI * f_temporal * time);

//        double st1 = even_spatial * bi_temporal;
//        double st3 = odd_spatial  * mono_temporal;

//        double st2 = even_spatial * mono_temporal;
//        double st4 = odd_spatial  * bi_temporal;

//        //Even and Odd components of spatio-temporal filters
//        st_even +=  st1 + st3;
//        st_odd  += -st2 + st4;

//    }

     //std::cout <<"Filter applied at : " << std::endl;
     //std::cout << "dx " << dx << " dy " << dy << " time " << time << std::endl;
     //std::cout << "orientation " << theta << " " << "phase " << phase << " " << std::endl;//Debug Code
     //std::cout << "Filter centered at X : " << center_x << " Y : "<< center_y << std::endl;//Debug Code

     //Spatial Filter computation
     double D = 2 * M_PI * (1/ pow(var_spatial,2));

     //CHANGED
     double dx_theta =  dx * cos(theta) + dy * sin(theta);
     double dy_theta = -dx * sin(theta) + dy * cos(theta);
     double gaussian_spatial = exp( - ( pow(dx_theta, 2) + pow(dy_theta,2) ) / ( 2 * pow(var_spatial,2) ) ); //double gaussian_spatial = exp( - ( pow(dx,2) + pow(dy,2) ) / ( 2 * pow(var_spatial,2) ) );
     double f1 = f_spatial * dx_theta; //double f1 = f_spatial * ( dx * cos(-theta) + dy * sin(-theta) );

     double even_spatial = D * gaussian_spatial  * cos( (2 * M_PI * f1 ) + phase );
     double odd_spatial = D * gaussian_spatial * sin( (2 * M_PI * f1) + phase );

     //Temporal Filter computation
     double gaussian_temporal = exp( - pow(time,2) / (2 * pow(var_temporal,2) )  );

     double mono_temporal = gaussian_temporal * cos( 2 * M_PI * f_temporal * time);
     double bi_temporal =  gaussian_temporal * sin( 2 * M_PI * f_temporal * time);

     double st1 = even_spatial * bi_temporal;
     double st3 = odd_spatial * mono_temporal;

     double st2 = even_spatial * mono_temporal;
     double st4 = odd_spatial * bi_temporal;

     //Even and Odd components of spatio-temporal filters
     double st_even = st1 + st3;
     double st_odd =  -st2 + st4; //CHECK THIS

     //std::cout << "Event Convolution computed: " << st_even << " "<<st_odd<< std::endl;//Debug Code

    return std::make_pair(st_even,st_odd);

}

//Destructor definition
stFilters::~stFilters(){

}
