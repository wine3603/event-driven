/*
 * spatialFilter.hpp
 *
 *  Created on: April 16, 2015
 *      Author: yeshasvi tvs
 */


#include <iostream>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <vector>
#include <malloc.h>
#include <memory.h>
#include <utility>
#include <iCub/emorph/all.h>
//#include <tuple>

using namespace std;

#ifndef STFILTERS_H
#define STFILTERS_H

class stFilters{

    public:

        //Data members
        //Spatial filter parameters
        double f_spatial;
        double var_spatial;

        //Temporal filter parameters
        double f_temporal;
        double var_temporal;

        //Member functions

        //Constructor definition
//        stFilters(double frequency_spatial = 0.05, double variance_spatial = 10, double frequency_temporal = 0.0625, double variance_temporal = 5){

//            std::cout<<"Spatial-Temporal Filter parameters initialization..."<<std::endl;


//            //Spatial filter parameters
//            f_spatial = frequency_spatial;
//            var_spatial = variance_spatial;
//            omega = 2 * M_PI * f_spatial; //Variable to compute disparity



//            //Temporal filter parameters
//            f_temporal = frequency_temporal;
//            var_temporal = variance_temporal;

//            std::cout<<"Parameters : "<< f_spatial << " " << var_spatial << " "<< f_temporal << " "<< var_temporal <<std::endl;//Debug Code


//        }

        //constructor
        stFilters();

        void setParams(double f_spatial, double var_spatial,
                       double f_temporal, double var_temporal);
        std::pair<double,double> filtering(int&, int&, double&, int &, double&, bool);

        //destructor
        virtual ~stFilters();


};



#endif // STFILTERS_H
