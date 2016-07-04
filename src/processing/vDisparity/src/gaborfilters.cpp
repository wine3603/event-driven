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

#include "gaborfilters.h"

gaborfilter::gaborfilter()
{
    cx = 0;
    cy = 0;

    orientation = 0;
    phase = 0;
    sigma = 0;
    fspatial = 1.0 / (2.0 * sigma);

    costheta = cos(orientation);
    sintheta = sin(orientation);
    neg2var = -2.0 * pow(sigma, 2.0);
    coscoeff = 2.0 * M_PI * fspatial;

    response = 0;
}

void gaborfilter::setCenter(int cx, int cy)
{
    this->cx = cx;
    this->cy = cy;
}

void gaborfilter::setParameters(double sigma, double orientation, double phase)
{
    this->sigma = sigma;
    this->fspatial = 1.0 / (4.0 * sigma);
    this->orientation = orientation;
    this->phase = phase;

    costheta = cos(orientation);
    sintheta = sin(orientation);
    neg2var = -2.0 * pow(sigma, 2.0);
    coscoeff = 2.0 * M_PI * fspatial;
}

void gaborfilter::process(emorph::vEvent &evt, double gain)
{
    emorph::AddressEvent *ae = evt.getAs<emorph::AddressEvent>();
    if(!ae) return;
    int dx = ae->getX() - cx;
    int dy = ae->getY() - cy;

    double dx_theta =  dx * costheta + dy * sintheta;
    double dy_theta = -dx * sintheta + dy * costheta;

    //gain is extra computation that should be the same for all filters
    //double gain = (1.0 / (2.0 * M_PI * pow(sigma, 2.0)));

    //the gaussian component assumes a circular Gaussian shape? should it be an elipse instead?
    //double gaussianComponent = exp( (pow(dx_theta, 2.0) + pow(dy_theta, 2.0)) / neg2var );
    double gaussianComponent = exp( (pow(dx_theta, 2.0) + pow(dy_theta, 2.0)) / neg2var );
    //add in the even component also
    double cosComponent = 1.0;
    if(ae->getChannel())
        cosComponent = cos( (coscoeff * dx_theta ) + phase );
    else
        cosComponent = cos( (coscoeff * dx_theta ) );

    response += gain * gaussianComponent * cosComponent;
}

void gaborfilter::process(emorph::vQueue &q, double gain)
{
    for(emorph::vQueue::iterator wi = q.begin(); wi != q.end(); wi++)
        process(**wi, gain);
}

//empty line to make gcc happy
