

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org 
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

/**
 * @file lfCollectorThread.cpp
 * @brief Implementation of the thread (see header lfCollectorThread.h)
 */

#include <iCub/lfCollectorThread.h>
#include <cstring>
#include <cassert>
#include <cstdlib>
#include <time.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define COUNTERRATIO 1.25       //1.25 is the ratio 0.160/0.128
#define MAXVALUE 4294967295
#define THRATE 10
#define STAMPINFRAME  // 10 ms of period times the us in 1 millisecond + time for computing
#define retinalSize 128
#define CHUNKSIZE 2048
#define dim_window 5
#define synch_time 1000

lfCollectorThread::lfCollectorThread() : RateThread(THRATE) {
    synchronised = false;
    greaterHalf = false;
    firstRun = true;
    count=0;
    minCount = 0; //initialisation of the timestamp limits of the first frame
    idle = false;
    bufferCopy = (char*) malloc(CHUNKSIZE);
    //bufferRead = (char*) malloc(8192);
    countStop = 0;
    verb = false;
}

lfCollectorThread::~lfCollectorThread() {
    printf("freeing memory in collector");
    delete bufferCopy;
}

bool lfCollectorThread::threadInit() {
    printf(" \nstarting the threads.... \n");
    //outPort.open(getName("/left:o").c_str());
    //outPortRight.open(getName("/right:o").c_str());

    resize(retinalSize, retinalSize);
    printf("starting the converter!!!.... \n");
    cfConverter=new cFrameConverter();
    cfConverter->useCallback();
    cfConverter->open(getName("/retina:i").c_str());
    printf("\n opening retina\n");
    printf("starting the plotter \n");
    pThread = new plotterThread();
    pThread->setName(getName("").c_str());
    pThread->start();


    //minCount = cfConverter->getEldestTimeStamp();
    startTimer = Time::now();
    //clock(); //startTime ;
    //T1 = times(&start_time);
    //microseconds = 0;
    //microsecondsPrev = 0;
    gettimeofday(&tvend, NULL);
    unmask_events.start();
    count = 0;
    microsecondsPrev = 0;
    return true;
}

void lfCollectorThread::interrupt() {
    outPort.interrupt();
}

void lfCollectorThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string lfCollectorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void lfCollectorThread::resize(int widthp, int heightp) {
    imageLeft = new ImageOf<PixelMono>;
    imageLeft->resize(widthp,heightp);
    imageRight = new ImageOf<PixelMono>;
    imageRight->resize(widthp,heightp);
}


void lfCollectorThread::getMonoImage(ImageOf<yarp::sig::PixelMono>* image, unsigned long minCount,unsigned long maxCount, bool camera){
    assert(image!=0);
    //image->resize(retinalSize,retinalSize);
    unsigned char* pImage = image->getRawImage();
    int imagePadding = image->getPadding();
    int imageRowSize = image->getRowSize();
    
    /*
    unsigned long int lasttimestamp = getLastTimeStamp();
    if (lasttimestamp == previousTimeStamp) {   //condition where there were not event between this call and the previous
        for(int r = 0 ; r < retinalSize ; r++){
            for(int c = 0 ; c < retinalSize ; c++) {
                *pImage++ = (unsigned char) 127;
            }
            pImage+=imagePadding;
        }
        return;
    }
    previousTimeStamp = lasttimestamp;
    */

    // determining whether the camera is left or right
    int* pBuffer = unmask_events.getEventBuffer(camera);
    unsigned long* pTime   = unmask_events.getTimeBuffer(camera);
    
    //printf("timestamp: min %d    max %d  \n", minCount, maxCount);
    //pBuffer += retinalSize * retinalSize - 1;
    for(int r = 0 ; r < retinalSize ; r++){
        for(int c = 0 ; c < retinalSize ; c++) {
            //drawing the retina and the rest of the image separately
            int value = *pBuffer;
            unsigned long timestampactual = *pTime;
            if (((timestampactual * COUNTERRATIO) > minCount)&&((timestampactual * COUNTERRATIO) < maxCount)) {   //(timestampactual != lasttimestamp)
                *pImage++ = (unsigned char) 127 + value;
               
            }
            else {
                *pImage++ = (unsigned char) 127;
               
                }
            pBuffer ++;
            pTime ++;
        }
        pImage+=imagePadding;
    }
    //unmask_events.setLastTimestamp(0);
}



void lfCollectorThread::run() {
    count++;
    if(!idle) {
        
        // reads the buffer received
        //bufferRead = cfConverter->getBuffer();    
        // saves it into a working buffer
        cfConverter->copyChunk(bufferCopy);//memcpy(bufferCopy, bufferRead, 8192);
        //printf("returned 0x%x \n", bufferCopy);
        // extract a chunk/unmask the chunk
       
        unmask_events.unmaskData(bufferCopy, CHUNKSIZE,verb);
        if(verb) {
            verb = false;
            countStop = 0;
        }
        //printf("returned 0x%x \n", bufferCopy);


        //gettin the time between two threads
        gettimeofday(&tvend, NULL);
        //Tnow = ((u64)tvend.tv_sec) * 1000000 + ((u64)tvstart.tv_usec);
        Tnow = ((tvend.tv_sec * 1000000 + tvend.tv_usec)
                - (tvstart.tv_sec * 1000000 + tvstart.tv_usec));
        //printf("timeofday>%ld\n",Tnow );
        gettimeofday(&tvstart, NULL);       
        endTimer = Time::now();
        double interval = (endTimer - startTimer) * 1000000; //interval in us
        startTimer = Time::now();
        
        unsigned long int lastleft = unmask_events.getLastTimestamp();
        lc = lastleft * COUNTERRATIO; 
        unsigned long int lastright = unmask_events.getLastTimestampRight();
        rc = lastright * COUNTERRATIO;

	if((lc >= 4294967295) || (rc >= 4294967295)) {
	  verb = true;
	  printf("wrapping %d, %d \n",lc,rc );
	}

 
        
        //synchronising the threads at the connection time
        if ((cfConverter->isValid())&&(!synchronised)) {
	    printf("Sychronised Sychronised Sychronised Sychronised ");
            //firstRun = false;
            minCount = lc - interval * dim_window; 
            //cfConverter->getEldestTimeStamp();                                                                   
            minCountRight = rc - interval * dim_window;
            printf("synchronised %1f! %d,%d,%d||%d,%d,%d \n",interval, minCount, lc, maxCount, minCountRight, rc, maxCountRight);
            startTimer = Time::now();
            synchronised = true;
            //minCount = unmask_events.getLastTimestamp();
            //printf("minCount %d \n", minCount);
            //minCountRight = unmask_events.getLastTimestamp();
            count = synch_time - 200;
        }
 

        //synchronising the thread every time interval 1000*period of the thread
        //if (count % synch_time == 0) {
	if (count == synch_time) {
            minCount = lc - interval * dim_window; //cfConverter->getEldestTimeStamp();        
            minCountRight = rc - interval * dim_window; 
            printf("synchronised %1f! %d,%d,%d||%d,%d,%d \n",interval, minCount, lc, maxCount, minCountRight, rc, maxCountRight);
            startTimer = Time::now();
            synchronised = true; 
        }
        else {
            // this value is simply the ration between the timestamp reported by the aexGrabber (62.5Mhz) 
            //and the correct timestamp counter clock of FPGA (50 Mhz)
            microsecondsPrev = interval;
            interval = Tnow;
            minCount = minCount + interval ; // * (50.0 / 62.5) * 1.10;
            minCountRight = minCountRight + interval;
        }   
             
        maxCount =  minCount + interval * dim_window;
        maxCountRight =  minCountRight + interval * dim_window;
        
        if(count % 100 == 0) { 
	  //printf("countStop %d lcprev %d lc %d \n",countStop, lcprev,lc);
            if (lcprev == lc) { 
                countStop++;
		printf("countStop %d \n", countStop);
	    }            
            //else if (rcprev == rc) { 
            //    countStop++;
            //    printf("countStop %d \n", countStop);
            //}
	    else {
	      countStop--;
	      printf("countStop %d \n", countStop);
	      if(countStop<= 0)
		countStop = 0;
	    }
            lcprev = lc;
            rcprev = rc;
        }
        
        
	
	//resetting time stamps at overflow
        if (countStop == 10) {
            //printf("resetting time stamps!!!!!!!!!!!!! %d %d   \n ", minCount, minCountRight);
            //cfConverter->resetTimestamps(); 
            verb = true;
            printf("countStop %d %d \n",countStop, verb );
	    count = synch_time - 200;
        }
	

	


        getMonoImage(imageRight,minCount,maxCount,0);
        getMonoImage(imageLeft,minCountRight,maxCountRight,1);
        pThread->copyLeft(imageLeft);
        pThread->copyRight(imageRight);
    }
}


/*
void lfCollectorThread::run() {
    count++;
    if(count == 100000) {
        count = 0;
    }
    if(!synchronised) {
        countDivider = count; 
    }
    
    //T2 = times(&stop_time);
    unsigned long int lastleft = cfConverter->getLastTimeStamp();
    lc = lastleft * 1.25; //1.25 is the ratio 0.160/0.128
    unsigned long int lastright = cfConverter->getLastTimeStampRight();
    rc = lastright * 1.25;

    //gettimeofday(&tvend, NULL);
    //Tnow = ((u64)tvend.tv_sec) * 1000000 + ((u64)tvstart.tv_usec);
    
    endTimer = Time::now();
    double interval = (endTimer - startTimer) * 1000000; //interval in us
    startTimer = Time::now();

    //clock_gettime(CLOCK_REALTIME, &stop_time );
    //double diffTime = (endTime - startTime);
    //printf("timeofday>%ld\n", ((tvend.tv_sec * 1000000 + tvend.tv_usec)
	//	  - (tvstart.tv_sec * 1000000 + tvstart.tv_usec)));
   
    //double time = (double)stop_time.tms_utime - start_time.tms_utime;
    //microseconds = stop_time.tv_nsec / 1000 ; 
    
    //gettimeofday(&tvstart, NULL);
    //startTime = clock();
    //T1 = times(&start_time);
    //clock();
    //clock_gettime( CLOCK_REALTIME, &start_time );
    
    
    if ((cfConverter->getInputCount()) && (!synchronised)) { 
        minCount = lc - interval * 2; //cfConverter->getEldestTimeStamp();        
        minCountRight = rc - interval * 2;
        printf("synchronised! %d,%d,%d||%d,%d,%d \n", minCount, lc, maxCount, minCountRight, rc, maxCountRight);
        startTimer = Time::now();
        synchronised = true;    
    }
    else if (count % 1000 == 0) {
        minCount = lc - interval * 2; //cfConverter->getEldestTimeStamp();        
        minCountRight = rc - interval * 2; 
        printf("synchronised! %d,%d,%d||%d,%d,%d \n", minCount, lc, maxCount, minCountRight, rc, maxCountRight);
        startTimer = Time::now();
        synchronised = true;  
    }
    else {
        minCount = minCount + interval ; // * (50.0 / 62.5) * 1.10;
        minCountRight = minCountRight + interval;
    }

    if ((lc > 10000000)||(rc>10000000)) {
        greaterHalf = true;
    }
    else if(((lc < 10000000)||(rc < 10000000))&&(greaterHalf)) {
        greaterHalf = false;
        cfConverter->resetTimestamps();
        printf("resetting time stamps!!!!!!!!!!!!!");
    }

             
    // this value is simply the ration between the timestamp reported by the aexGrabber (62.5Mhz) 
    //and the correct timestamp counter clock of FPGA (50 Mhz)
    maxCount =  minCount + interval * 5;
    maxCountRight =  minCountRight + interval * 5;
    if( count % 100 == 0) {
        printf("greterHalf:%d! %d,%d,%d||%d,%d,%d \n",greaterHalf, minCount, lc, maxCount, minCountRight, rc, maxCountRight);
    }
    precl = lc;

    
    microsecondsPrev = microseconds;
    if(outPort.getOutputCount()) {
        ImageOf<yarp::sig::PixelMono>& outputImage=outPort.prepare();
        if(&outputImage!=0) {
            cfConverter->getMonoImage(&outputImage, minCount, maxCount,1);
            outPort.write();
        }
        else {
            printf("reference to the outimage null \n");
        }
    }

    if(outPortRight.getOutputCount()) {
        ImageOf<yarp::sig::PixelMono>& outputImageRight=outPortRight.prepare();
        if(&outputImageRight!=0) {
            cfConverter->getMonoImage(&outputImageRight, minCountRight, maxCountRight, 0);
            outPortRight.write();
        }
        else {
            printf("reference to the outimage null \n");
        }
    }
    //minCount = cfConverter->getLastTimeStamp(); //get the last before going to sleep
    
}
*/

void lfCollectorThread::threadRelease() {
    idle = false;
    printf("Threadrelease:freeing bufferCopy \n");
    //free(bufferCopy);
    printf("Threadrelease:closing ports \n");
    outPort.close();
    outPortRight.close();
    delete imageLeft;
    delete imageRight;
    printf("Threadrelease         stopping plotterThread \n");
    pThread->stop();
    printf("Threadrelease         deleting converter \n");
    delete cfConverter;
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------
