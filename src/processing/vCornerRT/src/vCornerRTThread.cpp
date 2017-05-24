#include "vCornerRTThread.h"

using namespace ev;

vCornerThread::vCornerThread(unsigned int height, unsigned int width, std::string name, bool strict, int qlen,
                             int windowRad, int sobelsize, double sigma, double thresh, int nthreads)
{
    this->height = height;
    this->width = width;
    this->name = name;
    this->strict = strict;
    this->qlen = qlen;
    this->windowRad = windowRad;
//    this->sobelsize = sobelsize;
//    this->sigma = sigma;
//    this->thresh = thresh;
    this->nthreads = nthreads;

    std::cout << "Creating surfaces..." << std::endl;
    surfaceleft.initialise(height, width);
    surfaceright.initialise(height, width);

//    int gaussiansize = 2*windowRad + 2 - sobelsize;
//    convolution.configure(sobelsize, gaussiansize);
//    convolution.setSobelFilters();
//    convolution.setGaussianFilter(sigma);

    std::cout << "Using a " << sobelsize << "x" << sobelsize << " filter ";
    std::cout << "and a " << 2*windowRad + 1 << "x" << 2*windowRad + 1 << " spatial window" << std::endl;

    spfilter.initialise(width, height, 100000, 1);

    for(int i = 0; i < nthreads; i ++) {
        computeThreads.push_back(new vComputeThread(sobelsize, windowRad, sigma, thresh, qlen, &outthread));
    }
    std::cout << "Using " << nthreads << " threads for computation " << std::endl;

    this->cpudelay = 0.005;
    this->prevstamp = 0;
    this->t1 = this->t2 = 0.0; // yarp::os::Time::now();

}

bool vCornerThread::threadInit()
{

    if(!allocatorCallback.open("/" + name + "/vBottle:i")) {
    std::cout << "could not open vBottleIn port " << std::endl;
        return false;
    }

    if(!outthread.open("/" + name + "/vBottle:o")) {
        std::cout << "could not open vBottleOut port" << std::endl;
        return false;
    }
    if(!outthread.start())
        return false;

//    if(!vBottleOut.open("/" + name + "/vBottle:o")) {
//        std::cout << "could not open vBottleOut port" << std::endl;
//        return false;
//    }

    std::string debugPortName = "/" + name + "/score:o";
    if(!debugPort.open(debugPortName)) {
        std::cout << "could not open debug port" << std::endl;
        return false;
    }

    std::cout << "Thread initialised" << std::endl;
    return true;
}


void vCornerThread::onStop()
{
    allocatorCallback.close();
    allocatorCallback.releaseDataLock();

    for(int i = 0; i < nthreads; i++)
        delete computeThreads[i];
}


void vCornerThread::run()
{

    while(true) {

       // double t3 = yarp::os::Time::now();

        ev::vQueue *q = 0;
        while(!q && !isStopping()) {
            q = allocatorCallback.getNextQ(yarpstamp);
        }
        if(isStopping()) break;

       // cpudelay -= yarp::os::Time::now() - t3;

        int countProcessed = 0;
        for(ev::vQueue::iterator qi = q->begin(); qi != q->end(); qi++) {

            auto ae = ev::is_event<ev::AE>(*qi);
            double dt = ae->stamp - prevstamp;
            if(dt < 0.0) dt += vtsHelper::max_stamp;
            cpudelay -= dt * vtsHelper::tsscaler;
            prevstamp = ae->stamp;

//            //we need to filter the input as we will less likely process noise
//            if(!spfilter.check(ae->x, ae->y, ae->polarity, ae->channel, ae->stamp))
//                continue;

            ev::historicalSurface *cSurf;
            if(ae->getChannel() == 0)
                cSurf = &surfaceleft;
            else
                cSurf = &surfaceright;
            cSurf->addEvent(*qi);

            if(cpudelay < 0.0) cpudelay = 0.0;

            if(cpudelay <= 0.1) {
//                if(t1 == 0.0)
                    t1 = yarp::os::Time::now();

                //we look for a free thread
                for(int k = 0; k < nthreads; k++) {
                    if(!computeThreads[k]->isRunning()) {
//                        t1 = yarp::os::Time::now();
                        //vQueue subsurf = cSurf->getSurfaceN(0, qlen, windowRad, ae->x, ae->y);
//                        computeThreads[k]->setData(cSurf->getSurfaceN(0, 5, windowRad, ae->x, ae->y), yarpstamp);
                        computeThreads[k]->setData(cSurf, yarpstamp);
//                        cpudelay += yarp::os::Time::now() - t1;
                        computeThreads[k]->start();

                        countProcessed++;
//                        std::cout << k << std::endl;
                        break;
                    }
                }
                //if(!processing) std::cout << ".";

                //time it took to process
//                cpudelay = 0.0;
                cpudelay += yarp::os::Time::now() - t1;
//                t1 = yarp::os::Time::now();

            }

        }

        std::cout << q->size() << " " << countProcessed << " " << (double)countProcessed/q->size() << std::endl;

        //std::cout << std::endl;

        if(debugPort.getOutputCount()) {
            yarp::os::Bottle &scorebottleout = debugPort.prepare();
            scorebottleout.clear();
            scorebottleout.addDouble(cpudelay);
            debugPort.write();
        }

        allocatorCallback.scrapQ();
        //std::cout << allocatorCallback.scrapQ() << std::endl;

    }

}

//void vCornerThread::run()
//{

//    while(true) {

//        ev::vQueue *q = 0;
//        while(!q && !isStopping()) {
//            q = allocatorCallback.getNextQ(yarpstamp);
//        }
//        if(isStopping()) break;

////        ev::vBottle fillerbottle;
////        int count = 0;
//        for(ev::vQueue::iterator qi = q->begin(); qi != q->end(); qi++) {

//            auto ae = ev::is_event<ev::AE>(*qi);
//            double dt = ae->stamp - prevstamp;
//            if(dt < 0.0) dt += vtsHelper::max_stamp;
//            cpudelay -= dt * vtsHelper::tsscaler;
//            prevstamp = ae->stamp;

//            //we need to filter the input as we will less likely process noise
//            if(!spfilter.check(ae->x, ae->y, ae->polarity, ae->channel, ae->stamp))
//                continue;

//            ev::historicalSurface *cSurf;
//            if(ae->getChannel() == 0)
//                cSurf = &surfaceleft;
//            else if(ae->getChannel() == 1)
//                cSurf = &surfaceright;
//            cSurf->addEvent(ae);

//            if(cpudelay < 0.0) cpudelay = 0.0;

//            if(cpudelay <= 0.05) {
////                if(t1 == 0.0)
//                t1 = yarp::os::Time::now();

//                const vQueue subsurf = cSurf->getSurfaceN(0, qlen, windowRad, ae->x, ae->y);
//                bool isc = detectcorner(subsurf, ae->x, ae->y);

//                //if it's a corner, add it to the output bottle
//                if(isc) {
//                    auto ce = ev::make_event<LabelledAE>(ae);
//                    ce->ID = 1;
////                    fillerbottle.addEvent(ce);
//                    outthread.pushevent(ce, yarpstamp);
////                    count++;
//                }

//                //time it took to process
////                cpudelay = 0.0;
//                cpudelay += yarp::os::Time::now() - t1;
////                t1 = yarp::os::Time::now();

////                std::cout << count << std::endl;
//            }


//        }

//        if(debugPort.getOutputCount()) {
//            yarp::os::Bottle &scorebottleout = debugPort.prepare();
//            scorebottleout.clear();
//            scorebottleout.addDouble(cpudelay);
//            debugPort.write();
//        }

////        if( (yarp::os::Time::now() - t2) > 0.001 && fillerbottle.size() ) {
////            vBottleOut.setEnvelope(yarpstamp);
////            ev::vBottle &eventsout = vBottleOut.prepare();
////            eventsout.clear();
////            eventsout = fillerbottle;
////            vBottleOut.write(strict);
////            fillerbottle.clear();
////            t2 = yarp::os::Time::now();
////        }

//        allocatorCallback.scrapQ();

//    }

//}

//bool vCornerThread::detectcorner(ev::vQueue patch, int x, int y)
//{

//    //set the final response to be centred on the current event
//    convolution.setResponseCenter(x, y);

//    //update filter response
//    for(unsigned int i = 0; i < patch.size(); i++)
//    {
//        //events the patch
//        auto vi = ev::is_event<ev::AE>(patch[i]);
//        convolution.applysobel(vi);

//    }
//    convolution.applygaussian();

//    double score = convolution.getScore();

//    //reset responses
//    convolution.reset();

//    //if score > thresh tag ae as ce
//    return score > thresh;

//}

/*////////////////////////////////////////////////////////////////////////////*/
//threaded computation
/*////////////////////////////////////////////////////////////////////////////*/
vComputeThread::vComputeThread(int sobelsize, int windowRad, double sigma, double thresh, unsigned int qlen, collectorPort *outthread)
{
    this->sobelsize = sobelsize;
    this->windowRad = windowRad;
    this->sigma = sigma;
    this->thresh = thresh;
    this->qlen = qlen;
    int gaussiansize = 2*windowRad + 2 - sobelsize;
    convolution.configure(sobelsize, gaussiansize);
    convolution.setSobelFilters();
    convolution.setGaussianFilter(sigma);
    this->outthread = outthread;
}

void vComputeThread::setData(historicalSurface *cSurf, yarp::os::Stamp ystamp)
{
    patch.clear();
    cSurf->getSurfaceN(patch, 0, qlen, windowRad); // patch;
    this->ystamp = ystamp;
}

void vComputeThread::run()
{
//    std::cout << "running " << std::endl;
//    yarp::os::Time::delay(0.001);
    if(patch.size() == 0) return;

    auto aep = is_event<AE>(patch.front());
    if(detectcorner(aep->x, aep->y)) {
        ev::event<ev::LabelledAE> ce = make_event<LabelledAE>(aep);
        ce->ID = 1;
        outthread->pushevent(ce, ystamp);
    }

}

bool vComputeThread::detectcorner(int x, int y)
{

    //set the final response to be centred on the current event
    convolution.setResponseCenter(x, y);

    //update filter response
    for(unsigned int i = 0; i < patch.size(); i++)
    {
        //events the patch
        auto vi = is_event<AE>(patch[i]);
        convolution.applysobel(vi);

    }
    convolution.applygaussian();

    double score = convolution.getScore();

    //reset responses
    convolution.reset();

//    std::cout << x << " " << y << " " << score << std::endl;

    //if score > thresh tag ae as ce
    return score > thresh;

}
