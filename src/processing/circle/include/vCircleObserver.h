/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it
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

#include <iCub/emorph/all.h>
#include <opencv2/opencv.hpp>
#include <iCub/ctrl/kalman.h>

class vCircleObserver
{

public:

    //data
    emorph::vWindow *window;

    double cx, cy, cr;

    //parameters
    int width;
    int height;
    int sRadius;
    int windowSize;
    int iterations;
    int minVsReq4RANSAC;
    double inlierThreshold;

    //old parameters
    bool stepbystep;
    int tRadius;

    //private functions
    //void createLocalSearch(int x, int y);
    //void pointTrim(int x, int y);
    //void linearTrim(int x1, int y1, int x2, int y2);
    //cv::Mat createLocalActivityWindow(int x, int y);
    //double calculateCircleActivity(int cx, int cy, int r);

    bool calculateCircle(double x1, double x2, double x3,
                         double y1, double y2, double y3,
                         double &cx, double &cy, double &cr);


public:

    vCircleObserver();

    void addEvent(emorph::vEvent &event);
    double oneShotObserve(double &cx, double &cy, double &cr);
    double RANSAC(double &cx, double &cy, double &cr);
    int gradient(double &cx, double &cy, double &cr);
    int gradient2(double &cx, double &cy, double &cr);
    int gradientView();
    int gradientView2();
    int eigenView();

    //bool localCircleEstimate(emorph::AddressEvent &event, double &cx,
    //                        double &cy, double &cr, bool showDebug = false);

    void viewEvents();
    double globalInlierCount(double cx, double cy, double cr);
    //temporary debug stuff

};

/*//////////////////////////////////////////////////////////////////////////////
  CIRCLE TRACKER
  ////////////////////////////////////////////////////////////////////////////*/

class vCircleTracker
{
public:

    iCub::ctrl::Kalman *filter;
    emorph::vQueue zq;
    bool active;
    emorph::vtsHelper unwrap;
    double pTS;

    //system variance
    double svPos;
    double svSiz;

    //private functions
    bool makeObservation(double &cx, double &cy, double &cr);
    double Pvgd(double xv, double yv);

public:

    vCircleTracker(double svPos, double svSiz, double zvPos, double zvSiz);
    ~vCircleTracker();


    //add event will:
    //1. check closeness to distribution
    //2. update zq if < c
    //3. perform correction based on observation from zq
    //4. destroy if validation gate too high
    //5. return whether, updated, non-updated, needs destroying
    double addEvent(emorph::AddressEvent v, double cT);

    //update state
    //also update zq values based on velocity
    double predict(double dt);

    double correct(double xz, double yz, double rz, double dt);

    double init(double xz, double yz, double rz);

    double Pzgd(double xz, double yz, double rz);


};

