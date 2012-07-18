#ifndef TSOPTFLOWTHREAD_HPP
#define TSOPTFLOWTHREAD_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <list>

//#include <yarp/os/RateThread.h>
#include <yarp/os/Thread.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <gsl/gsl_sort.h>
#include <gsl/gsl_statistics.h>

#include "unmask.hpp"
#include "unmaskDVS128.hpp"
#include "unmaskICUB.hpp"

#include "VelocityBuffer.h"

//#define _DEBUG
#define THRATE 1
#define CONTRAST 15

typedef unsigned int uint;

//yarp::sig::Matrix operator-(double, yarp::sig::Matrix&);
//yarp::sig::Matrix operator%(yarp::sig::Matrix&, yarp::sig::Matrix&);

//class tsOptFlowThread:public yarp::os::RateThread
class tsOptFlowThread:public yarp::os::Thread
{
public:
    tsOptFlowThread(uint&, uint&, std::string&, uint&, uint&, uint&, double&, uint&, uint&, uint&, double&, double&, int&, uint&, bool&, yarp::sig::Matrix*, yarp::sig::Matrix*, yarp::os::Semaphore*, VelocityBuffer*);
    ~tsOptFlowThread();
    void setBuffer(char*, uint);
    void run();
private:

    /**
    *
    */
    void setSobelFilters(uint, yarp::sig::Matrix&, yarp::sig::Matrix&);
    int factorial(int);
    int Pasc(int, int);
    void updateAll();
    void compute();
    uint createPlan(yarp::sig::Matrix&);
    uint createPlanAndCompute(yarp::sig::Matrix&, double&, double&);
    uint createPlanAndCompute(yarp::sig::Matrix&, double&, double&, uint&, uint&, uint&);
    void flip();

    void printMatrix(yarp::sig::Matrix&);
/*
*   VARIABLE
*/
    double alpha;
    double threshold;
    double tauD;
    uint tsVal;
    uint neighbor;
    uint neighLR;
    uint height;
    uint width;
    uint sobelSz;
    uint sobelLR;
    uint accumulation;
    uint binAcc;
    bool saveOf;

    uint borneSupX;
    uint borneSupY;
    uint borneInfX;
    uint borneInfY;

    yarp::sig::Matrix sobelx;
    yarp::sig::Matrix sobely;

    yarp::sig::Matrix activity; double* activityData;
    yarp::sig::Matrix TSs;      double* TSsData;
    yarp::sig::Matrix TSs2Plan; double* TSs2PlanData;
    yarp::sig::Matrix compPurpTS;
    yarp::sig::Matrix subTSs;
    yarp::sig::Matrix alreadyComputedX;
    yarp::sig::Matrix alreadyComputedY;

    double  dx;
    double  dy;
    
/*** To compute the plan ***/
    yarp::sig::Matrix A;
    yarp::sig::Matrix At;
    yarp::sig::Matrix AtA;
    yarp::sig::Matrix ctrl;
    yarp::sig::Vector abc;
    yarp::sig::Vector Y;
/***************************/
    uint orientation;
    uint addrx, addrxBack;
    uint addry;
    uint eye;
    int polarity, polSel;
    uint timestamp;
    uint refts;
    bool first;
    
    unmask *unmasker;
    yarp::sig::Matrix *vxMat;
    yarp::sig::Matrix *vyMat;
    double *vxMatData;
    double *vyMatData;

    yarp::os::Semaphore *mutex;

    yarp::sig::Matrix binEvts;
    uint iBinEvts;

    yarp::sig::Matrix wMat;
    yarp::sig::Matrix sMat;
    uint iSMat;
    double *xNeighFlow;
    double *yNeighFlow;
    uint ixNeighFlow;
    uint iyNeighFlow;
    
    VelocityBuffer *velBuf;

    int *trans2neigh;
    int iT2N;

    ofstream saveFile;
    stringstream line2save;
};

#endif //TSOPTFLOWTHREAD_HPP