#include "vCornerRTThread.h"

using namespace ev;

vCornerThread::vCornerThread(unsigned int height, unsigned int width, std::string name, bool strict, int qlen,
                             int windowRad, int sobelsize, double sigma, double thresh)
{
    this->height = height;
    this->width = width;
    this->name = name;
    this->strict = strict;
    this->qlen = qlen;
    this->windowRad = windowRad;
    this->sobelsize = sobelsize;
    this->sigma = sigma;
    this->thresh = thresh;

    std::cout << "Creating surfaces..." << std::endl;
    surfaceleft.initialise(height, width);
    surfaceright.initialise(height, width);

    int gaussiansize = 2*windowRad + 2 - sobelsize;
    convolution.configure(sobelsize, gaussiansize);
    convolution.setSobelFilters();
    convolution.setGaussianFilter(sigma);

    std::cout << "Initialising thread..." << std::endl;
    std::cout << "Using a " << sobelsize << "x" << sobelsize << " filter ";
    std::cout << "and a " << 2*windowRad + 1 << "x" << 2*windowRad + 1 << " spatial window" << std::endl;

    this->cpudelay = 0.005;
    this->prevstamp = 0;
    this->t1 = this->t2 = yarp::os::Time::now();

}

bool vCornerThread::threadInit()
{

    if(!allocatorCallback.open("/" + name + "/vBottle:i"))
        return false;

    if(!vBottleOut.open("/" + name + "/vBottle:o")) {
        std::cout << "could not open vBottleOut port" << std::endl;
        return false;
    }

    std::string debugPortName = "/" + name + "/score:o";
    if(!debugPort.open(debugPortName)) {
        std::cout << "could not open debug port" << std::endl;
        return false;
    }

    std::cout << "Thread initialised" << std::endl;
    return true;
}

//bool vCornerThread::open(std::string portname)
//{
//    if(!allocatorCallback.open(portname))
//        return false;

//    start();
//    return true;
//}

void vCornerThread::onStop()
{
    allocatorCallback.close();
    allocatorCallback.releaseDataLock();
}


void vCornerThread::run()
{

    while(true) {

        ev::vQueue *q = 0;
        while(!q && !isStopping()) {
            q = allocatorCallback.getNextQ(yarpstamp);
        }
        if(isStopping()) break;

        ev::vBottle fillerbottle;
        for(ev::vQueue::iterator qi = q->begin(); qi != q->end(); qi++) {

            auto ae = ev::is_event<ev::AE>(*qi);
            double dt = ae->stamp - prevstamp;
            if(dt < 0.0) dt += vtsHelper::max_stamp;
            cpudelay -= dt * vtsHelper::tsscaler;
            prevstamp = ae->stamp;

            ev::historicalSurface *cSurf;
            if(ae->getChannel() == 0)
                cSurf = &surfaceleft;
            else if(ae->getChannel() == 1)
                cSurf = &surfaceright;
            cSurf->addEvent(ae);

            if(cpudelay <= 0.0) {
                if(t1 == 0.0)
                    t1 = yarp::os::Time::now();

                const vQueue subsurf = cSurf->getSurfaceN(0, qlen, windowRad, ae->x, ae->y);
                bool isc = detectcorner(subsurf, ae->x, ae->y);

                //if it's a corner, add it to the output bottle
                if(isc) {
                    auto ce = ev::make_event<LabelledAE>(ae);
                    ce->ID = 1;
                    fillerbottle.addEvent(ce);
                }

                //time it took to process
                cpudelay = 0.0;
                cpudelay += yarp::os::Time::now() - t1;
                t1 = yarp::os::Time::now();
            }

            if(debugPort.getOutputCount()) {
                yarp::os::Bottle &scorebottleout = debugPort.prepare();
                scorebottleout.clear();
                scorebottleout.addDouble(cpudelay);
                debugPort.write();
            }
        }

        if( (yarp::os::Time::now() - t2) > 0.001 && fillerbottle.size() ) {
            vBottleOut.setEnvelope(yarpstamp);
            ev::vBottle &eventsout = vBottleOut.prepare();
            eventsout.clear();
            eventsout = fillerbottle;
            vBottleOut.write(strict);
            fillerbottle.clear();
            t2 = yarp::os::Time::now();
        }

    }

    allocatorCallback.scrapQ();

}

bool vCornerThread::detectcorner(ev::vQueue patch, int x, int y)
{

    //set the final response to be centred on the current event
    convolution.setResponseCenter(x, y);

    //update filter response
    for(unsigned int i = 0; i < patch.size(); i++)
    {
        //events the patch
        auto vi = ev::is_event<ev::AE>(patch[i]);
        convolution.applysobel(vi);

    }
    convolution.applygaussian();

    double score = convolution.getScore();

    //reset responses
    convolution.reset();

    //if score > thresh tag ae as ce
    return score > thresh;

}
