// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
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
 * @file efExtractorThread.cpp
 * @brief Implementation of the thread (see header efExtractorThread.h)
 */

#include <iCub/efExtractorThread.h>
#include <iCub/cartesianFrameConverter.h>

#include <cxcore.h>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <cstring>
#include <cassert>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

#define DIM 10
#define THRATE 30
#define SHIFTCONST 100
#define RETINA_SIZE 128
#define FEATUR_SIZE 32
#define ADDRESS 0x40000
#define X_MASK 0x000000FE
#define X_MASK_DEC    254
#define X_SHIFT 1
#define Y_MASK 0x00007F00
#define Y_MASK_DEC  32512    
#define Y_SHIFT 8
#define POLARITY_MASK 0x00000001
#define POLARITY_SHIFT 0
#define CONST_DECREMENT 10

#define CHUNKSIZE 32768

#define VERBOSE

inline int convertChar2Dec(char value) {
    if (value > 60)
        return value - 97 + 10;
    else
        return value - 48;
}

inline void copy_8u_C1R(ImageOf<PixelMono>* src, ImageOf<PixelMono>* dest) {
    int padding = src->getPadding();
    int channels = src->getPixelCode();
    int width = src->width();
    int height = src->height();
    unsigned char* psrc = src->getRawImage();
    unsigned char* pdest = dest->getRawImage();
    for (int r=0; r < height; r++) {
        for (int c=0; c < width; c++) {
            *pdest++ = (unsigned char) *psrc++;
        }
        pdest += padding;
        psrc += padding;
    }
}

inline void copy_8u_C3R(ImageOf<PixelRgb>* src, ImageOf<PixelRgb>* dest) {
    int padding = src->getPadding();
    int channels = src->getPixelCode();
    int width = src->width();
    int height = src->height();
    unsigned char* psrc = src->getRawImage();
    unsigned char* pdest = dest->getRawImage();
    for (int r=0; r < height; r++) {
        for (int c=0; c < width; c++) {
            *pdest++ = (unsigned char) *psrc++;
            *pdest++ = (unsigned char) *psrc++;
            *pdest++ = (unsigned char) *psrc++;
        }
        pdest += padding;
        psrc += padding;
    }
}

efExtractorThread::efExtractorThread() : RateThread(THRATE) {
    resized = false;
    count             = 0;
    countEvent        = 0;
    leftInputImage    = 0;
    rightInputImage   = 0;
    shiftValue        = 20;
    lastTimestampLeft = 0;

    idle = false;
    bufferCopy = (char*) malloc(CHUNKSIZE);
}

efExtractorThread::~efExtractorThread() {
    free(bufferCopy);
}

bool efExtractorThread::threadInit() {
    printf("starting the thread.... \n");
    /* open ports */
    printf("opening ports.... \n");
    outLeftPort.open(getName("/edgesLeft:o").c_str());
    outRightPort.open(getName("/edgesRight:o").c_str());
    inLeftPort.open(getName("/left:i").c_str());
    inRightPort.open(getName("/right:i").c_str());
    outEventPort.open(getName("/event:o").c_str());

    cfConverter=new cFrameConverter();
    cfConverter->useCallback();
    cfConverter->open(getName("/retina:i").c_str());

    int SIZE_OF_EVENT = CHUNKSIZE >> 3; // every event is composed by 4bytes address and 4 bytes timestamp
    monBufSize_b = SIZE_OF_EVENT * sizeof(struct aer);

    bufferFEA = (aer *)  malloc(monBufSize_b);
    if ( bufferFEA == NULL ) {
        printf("bufferFEA malloc failed \n");
    }
    else {
        printf("bufferFEA successfully created \n");
    }

    eventFeaBuffer = new AER_struct[CHUNKSIZE>>3];

    printf("allocating memory for the LUT \n");
    lut = new int[RETINA_SIZE * RETINA_SIZE * 5];
    printf("initialising memory for LUT \n");

    for(int i = 0; i < RETINA_SIZE * RETINA_SIZE * 5; i++) {
        lut[i] = -1;
        //printf("i: %d  value : %d \n",i,lut[i]);
    }
    printf("successfully initialised memory for LUT \n");
    
    /*opening the file of the mapping and creating the LUT*/
    pFile = fopen (mapURL.c_str(),"rb");    
    fout  = fopen ("lut.txt","w+");
    fdebug  = fopen ("dumpSet.txt","w+");
    if (pFile!=NULL) {
        long lSize;
        size_t result;        
        // obtain file size:
        fseek (pFile , 0 , SEEK_END);
        lSize = ftell (pFile);
        printf("dimension of the file %lu \n", lSize);
        rewind(pFile);
        // saving into the buffer
        char * buffer;
        buffer = (char*) malloc (sizeof(char) * lSize);
        //fputs ("fopen example",pFile);
        printf("The file was correctly opened \n");
        result = fread (buffer,1,lSize,pFile);        
        printf(" Saved the data of the file into the buffer lSize:%lu \n", lSize);
        // checking the content of the buffer
        long word = 0;
        long input = -1, output = -1;
        short x, y;
        countMap = 0;
        //lSize
        for (int i = 0; i < lSize; i++) {            
            
            int value = convertChar2Dec(*buffer);
            //looking for EOL
            if(*buffer == 10) {
                //sac words
                if(word < 500000) {
                    word -= 262144;
                }
                else if(word < 1000000) {
                    word -= 524288;
                }    
                else {
                    word -= 1114112;
                }

                x = word & 0x001F;
                y = word >> 5;
                //printf("sac output: %d --> %d %d \n", word, x, y);
                output = y * 32 + x;
                word = 0;
            }
            //looking for space
            else if(*buffer == 32)  {
                //angel words
                if(word < 500000) {
                    word -= 262144;
                }
                else if(word < 1000000) {
                    word -= 524288;
                }    
                else {
                    word -= 1114112;
                }                
                
                x = (word & X_MASK) >> X_SHIFT;
                y = (word & Y_MASK) >> Y_SHIFT;
                //printf("angel input: %d --> %d %d \n", word, x, y);
                input = y * 128 + x;
                word = 0;
            }
            else{
                //printf("%d,%d ", value, word );
                word = word << 4;
                word += value;
            }
            buffer++;
            if((input != -1) && (output!=-1)) {
                
                int inputy  = input / 128;
                int inputx  = input - inputy * 128;
                int outputy = output / 32;
                int outputx = output - outputy * 32;
                
                bool continueSaving = true;

                // any input coordinate can point up to 5 output coordinates
                int i = 0;
                while(continueSaving) {
                    if(lut[input + i * RETINA_SIZE * RETINA_SIZE] != -1) {
                        if(lut[input + i * RETINA_SIZE * RETINA_SIZE ]!= output) { 
                            i++;
                            if (i>= 5) {
                                continueSaving = false;
                            }                                                      
                        }
                        else {
                            continueSaving = false;
                        }
                    }
                    else {
                        //saving
                        lut[input + i * RETINA_SIZE * RETINA_SIZE] = output;
                        printf("lut : %d-->%d (%d) \n", input, output, i);
                        fprintf(fout," %d %d > %d %d > %d %d   \n",input, output, inputy, inputx,  outputy, outputx);
                        input  = -1;
                        output = -1;
                        continueSaving = false;
                        countMap++;
                    }
                } //end of while
            }            
        }
        printf("counted the number of mapping %d \n", countMap);
    }
    
    leftInputImage  = new ImageOf<PixelMono>;
    leftOutputImage = new ImageOf<PixelMono>;
    //leftInputImage->resize(FEATUR_SIZE, FEATUR_SIZE);
    leftInputImage->resize(128,128);
    leftOutputImage->resize(128,128);
    int padding = leftInputImage->getPadding();
    //initialisation of the memory image
    unsigned char* pLeft   = leftInputImage->getRawImage();
    unsigned char* pLeftOut = leftOutputImage->getRawImage();
    int rowsize = leftInputImage->getRowSize();
    for(int r = 0 ; r < 128 ; r++){
        for(int c = 0 ; c < 128 ; c++){
            *pLeft  = 127; pLeft++;
            *pLeftOut = 127; pLeftOut++;
        }
        pLeft +=  padding;
        pLeftOut += padding;
    }
    
    printf("initialisation correctly ended \n");
    return true;
}

void efExtractorThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string efExtractorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void efExtractorThread::resize(int widthp, int heightp) {
    width = widthp;
    height = heightp;
    //leftInputImage = new ImageOf<PixelMono>;
    //leftInputImage->resize(width, height);
    //rightInputImage = new ImageOf<PixelMono>;
    //rightInputImage->resize(width, height);
}

void efExtractorThread::run() {   
    count++;
    countEvent = 0;
    //printf("counter %d \n", count);
    bool flagCopy;
    if(!idle) {
        
        //ImageOf<PixelMono> &left = outLeftPort.prepare();  
        //left.resize(128, 128);
        //left.zero();
        unsigned char* pLeft = leftOutputImage->getRawImage(); 
        
        //left.zero();
        
        //for(int r = 0 ; r < FEATUR_SIZE ; r++){
        //    for(int c = 0 ; c < FEATUR_SIZE ; c++){
        //        left(r,c) = 127;
        //    }
        //}
        
        // reads the buffer received
        // saves it into a working buffer        
        //printf("returned 0x%x 0x%x \n", bufferCopy, flagCopy);
        cfConverter->copyChunk(bufferCopy);
        //int unreadDim = selectUnreadBuffer(bufferCopy, flagCopy, resultCopy);

        //printf("Unmasking events:  %d \n", unreadDim);
        // extract a chunk/unmask the chunk       
        unmask_events.unmaskData(bufferCopy,CHUNKSIZE,eventFeaBuffer);
        
        //storing the response in a temp. image
        AER_struct* iterEvent = eventFeaBuffer;  //<------------ using the pointer to the unmasked events!
        
        unsigned char* pMemL = leftInputImage->getRawImage();
        int          rowSize = leftOutputImage->getRowSize();
        unsigned long ts;
        short pol, cam;
        //aer* bufferFEA_copy = bufferFEA;
        // visiting a discrete number of events
        int countUnmapped = 0;
        
        for(int i = 0; i < CHUNKSIZE>>3; i++ ) {
            ts  = iterEvent->ts;
            pol = iterEvent->pol;
            cam = iterEvent->cam;
            if(cam==1) {
                if(ts > lastTimestampLeft) {
                    lastTimestampLeft = ts;
                    printf(" %d %d \n",iterEvent->x,iterEvent->y );
                    int x = RETINA_SIZE - iterEvent->x;
                    int y = RETINA_SIZE - iterEvent->y;
                    //if((x >127)||(x<0)||(y>127)||(y<0)) break;
                    int posImage     = 0 * rowSize + 0;
                
                    if (outLeftPort.getOutputCount()){
                    
                        int padding    = leftOutputImage->getPadding();
                        int rowsizeOut = leftOutputImage->getRowSize();
                        float lambda   = 1.0;
                        int deviance   = 10;
                        //creating the debug image
                        if((pol > 0)&&(pLeft[posImage] <= (255 - deviance))){
                            //pLeft[pos] = 0.6 * pLeft[pos] + 0.4 * (pLeft[pos] + 40);
                            pLeft[posImage] =  (unsigned int) pLeft[posImage] + deviance;
                            //pMemL[pos] = (int)((1 - lambda) * (double) pMemL[pos] + lambda * (double) (pMemL[pos] + deviance));
                            //pLeft[posImage] = pMemL[posImage];
                            
                            //  if(pMemL[pos] > 127 + 70) {
                            //  bufferFEA_copy->address   = (u32) blob;
                            //  bufferFEA_copy->timestamp = (u32) ts;
                            //  #ifdef VERBOSE
                            //  fprintf(fdebug,"%08X %08X \n",blob,ts);  
                            //  #endif
                              
                              //fprintf(fout,"%08X %08X \n",bufferFEA_copy->address,ts);
                            //  bufferFEA_copy++; // jumping to the next event(u32,u32)
                            //  countEvent++;
                             // }
                            
                        }
                        if((pol<0)&&(pLeft[posImage] >= deviance)){ 
                            //pLeft[pos] = 0.6 * pLeft[pos] + 0.4 * (pLeft[pos] - 40);
                            //pMemL[pos] = (int) ((1 - lambda) * (double) pMemL[pos] + lambda * (double)(pMemL[pos] - deviance));
                            pLeft[posImage] =  (unsigned int) pLeft[posImage] - deviance;
                            //pLeft[posImage] = pMemL[posImage];
                            
                            //  if(pMemL[pos] < 127 - 70) {
                            //  bufferFEA_copy->address   = (u32) blob;
                            //  bufferFEA_copy->timestamp = (u32) ts;                
                            //  #ifdef VERBOSE
                            //  fprintf(fdebug,"%08X %08X \n",blob,ts);  
                            //  #endif                                
                              //fprintf(fout,"%08X %08X \n",bufferFEA_copy->address,ts);
                            //  bufferFEA_copy++; // jumping to the next event(u32,u32)
                             // countEvent++; 
                             // }
                            
                        }
                    
                        outLeftPort.prepare() = *leftOutputImage;
                        outLeftPort.write();
                    }
                }
                
                
                //printf("pointing in the lut at position %d %d %d \n",x, y, x + y * RETINA_SIZE);
                // extra output positions are pointed by i

                
                //printf("\n");
                iterEvent++;            
                ////end of if
            } //end of for i
            


            
            /*
            // leaking section of the algorithm    
            pMemL  = leftInputImage->getRawImage();
            pLeft  = leftOutputImage->getRawImage();
            int padding = leftOutputImage->getPadding();
            for (int i = 0; i< FEATUR_SIZE * FEATUR_SIZE; i++) {
                if(*pLeft >= 127 + CONST_DECREMENT) {                
                    *pLeft -= CONST_DECREMENT;
                }
                else if(*pLeft <= 127 - CONST_DECREMENT) {
                    *pLeft += CONST_DECREMENT;
                }
                else{
                    *pLeft = 127;
                }
                
                pLeft++;
                //*pLeft = *pMemL;
                //pLeft++;
            }
            //pMemL += padding;                
            pLeft += padding;
            */
            
            
           
        } //end if
        
        /*
        //building feature events
        char* buffer = (char*) bufferFEA;
        int sz = countEvent * sizeof(aer);
        // sending events 
        if (outEventPort.getOutputCount()) {            
        //printf("Sending \n");
        eventBuffer data2send(buffer,sz);    
        eventBuffer& tmp = outEventPort.prepare();
        tmp = data2send;
        outEventPort.write();
        }
        */
    } //end of idle
}



/*
#ifdef COMMENT                
                  for (int i = 0; i< 5 ; i++) {                
                  int pos      = lut[i * RETINA_SIZE * RETINA_SIZE +  y * RETINA_SIZE + x ];                        
                  
                  //printf("        pos ; %d ", pos);
                  if(pos == -1) {
                  if (x % 2 == 1) {
                  //printf("not mapped event %d \n",x + y * RETINA_SIZE);
                  countUnmapped++;
                  }                
                  }
                  else {
                  //creating an event
                  
                  int xevent_tmp   = pos / FEATUR_SIZE;
                  int xevent       = FEATUR_SIZE - xevent_tmp;
                  int yevent_tmp   = pos - xevent_tmp * FEATUR_SIZE;
                  int yevent       = yevent_tmp;
                  int polevent     = pol < 0 ? 0 : 1;
                  int cameraevent  = 0;
                  unsigned long blob = 0;
                  int posImage     = y * rowSize + x;
                  
                  //if(ts!=0) {
                  //    printf(" %d %d %d %08x %08x \n",pos, yevent, xevent,blob, ts);
                  //}
                  unmask_events.maskEvent(xevent, yevent, polevent, cameraevent, blob);
                  //unsigned long evPU;
                  //evPU = 0;
                  //evPU = evPU | polevent;
                  //evPU = evPU | (xevent << 1);
                  //evPU = evPU | (yevent << 8);
                  //evPU = evPU | (cameraevent << 15);
                  
                  //blob = blob & 0x0000FFFF;
                  //blob = 0x00001021;
                  //printf("Given pos %d extracts x=%d and y=%d pol=%d blob=%08x evPU = %08x \n", pos, xevent, yevent, pol, blob, ts);
                  
                  //
                  //bufferFEA_copy->address   = (u32) blob;
                  //bufferFEA_copy->timestamp = (u32) ts;                
                  ////fprintf(fout,"%08X %08X \n",bufferFEA_copy->address,ts);
                  //bufferFEA_copy++; // jumping to the next event(u32,u32)
                  //countEvent++;
                  //
                  
                  //pLeft = pointer to the left image
                  //pMem  = pointer to the left input image
                  
                  
                  } //end else
                  
                  } //end for i 5
#ifdef
*/


void efExtractorThread::interrupt() {
    outLeftPort.interrupt();
    outRightPort.interrupt();
    inLeftPort.interrupt();
    inRightPort.interrupt();
    outEventPort.interrupt();
}

void efExtractorThread::threadRelease() {
    /* closing the ports*/
    outLeftPort.close();
    outRightPort.close();
    inLeftPort.close();
    inRightPort.close();
    outEventPort.close();

    delete[] eventFeaBuffer;
    delete bufferFEA;
    delete cfConverter;
    
    /* closing the file */
    delete[] lut;
    fclose (pFile);
}

