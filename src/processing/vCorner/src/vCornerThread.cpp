#include "vCornerThread.h"

using namespace ev;

vCornerThread::vCornerThread(unsigned int height, unsigned int width, std::string name, bool strict, int eventsInPatch,
                             int windowRad, int sobelsize, double sigma, double thresh, int nthreads)
{
    this->height = height;
    this->width = width;
    this->name = name;
    this->strict = strict;
    this->eventsInPatch = eventsInPatch;
    this->windowRad = windowRad;
    this->sobelsize = sobelsize;
    channel = 1;
    this->sigma = sigma;
    this->thresh = thresh;
    this->nthreads = nthreads;

    eventhandler.configure(height, width, 0.1, eventsInPatch);

//    int gaussiansize = 2*windowRad + 2 - sobelsize;
//    convolution.setSobelFilters(sobelsize);
//    convolution.setGaussianFilter(sigma, gaussiansize);

}

bool vCornerThread::threadInit()
{   

    std::cout << "Initialising thread..." << std::endl;
    std::cout << "Using a " << sobelsize << "x" << sobelsize << " filter ";
    std::cout << "and a " << 2*windowRad + 1 << "x" << 2*windowRad + 1 << " spatial window" << std::endl;
    std::cout << "Using " << nthreads << " threads for the computation" << std::endl;

    if(!debugOut.open("/" + name + "/debug:o")) {
        std::cout << "could not open debug port" << std::endl;
        return false;
    }

    if(!vBottleOut.open("/" + name + "/vBottle:o")) {
        std::cout << "coult not open vBottleOut port" << std::endl;
        return false;
    }

    if(!eventhandler.open("/" + name + "/vBottle:i")) {
        std::cout << "Could not open eventhandler port" << std::endl;
        return false;
    }

    for(int i = 0; i < nthreads; i ++) {
        computeThreads.push_back(new vComputeThread(sobelsize, windowRad, sigma, thresh));
    }

    std::cout << "Thread initialised" << std::endl;
    return true;
}

void vCornerThread::run()
{
    std::cout << "Starting thread..." << std::endl;
    vComputeThread *cthread = 0;
    ev::vQueue roi;
    ev::vBottle fillerbottle;

    int countThreads = 0;
    while(!isStopping()) {

        //set cthread to point to the current thread
        cthread = computeThreads[countThreads];

        //wait the current thread to finish
//        std::cout << "stopping comuptation for thread " << countThreads << std::endl;
        cthread->join();
        if(!cthread->join())
            std::cout << "cannot stop thread " << countThreads << std::endl;


        //push the current thread response to the output bottle
        ev::event<ev::LabelledAE> ce = cthread->getResponse();
//        std::cout << "getting response from thread " << countThreads << std::endl;
        if(ce) fillerbottle.addEvent(ce);

        //get data and let the current thread process it
//        std::cout << "querying roi " << std::endl;
        roi = eventhandler.queryROI(channel, eventsInPatch, windowRad);

//        std::cout << "setting roi to thread " << countThreads << std::endl;
        cthread->setData(&roi);
//        std::cout << "data set" << std::endl;
        cthread->start();
//        std::cout << "started computation for thread " << countThreads << std::endl << std::endl;

        countThreads++;
        if(countThreads == nthreads) countThreads = 0;

        //write on output bottle
        double deltat = (yarp::os::Time::now() - prevTime);
        if(deltat > 0.01 && fillerbottle.size()) {
            yarp::os::Stamp yarpstamp = eventhandler.queryYstamp();
            vBottleOut.setEnvelope(yarpstamp);
            ev::vBottle &eventsout = vBottleOut.prepare();
            eventsout.clear();
            eventsout = fillerbottle;
            vBottleOut.write(strict);
            fillerbottle.clear();

            prevTime = yarp::os::Time::now();
        }
    }


//    vQueue patch;
//    ev::vBottle fillerbottle;

//    int count = 0;
//    while(!isStopping()) {
//        yarp::os::Time::delay(0.0001);

//        patch = eventhandler.queryROI(channel, eventsInPatch, windowRad);
//        if(!patch.size()) continue;

//        auto aep = is_event<AE>(patch.front());
//        bool isc = detectcorner(patch, aep->x, aep->y);

//        if(isc) {
//            auto ce = make_event<LabelledAE>(aep);
//            ce->ID = 1;
//            fillerbottle.addEvent(ce);
////            eventsout.addEvent(ce);
//            count++;
//        }

//        double deltat = (yarp::os::Time::now() - prevTime);
//        if(deltat > 0.01 && fillerbottle.size()) {
////            std::cout << "sending out " << std::endl;

//            yarp::os::Stamp yarpstamp = eventhandler.queryYstamp();
//            vBottleOut.setEnvelope(yarpstamp);
//            ev::vBottle &eventsout = vBottleOut.prepare();
//            eventsout.clear();
//            eventsout = fillerbottle;
//            vBottleOut.write(strict);
//            fillerbottle.clear();

//            //            eventsout.clear();

//            prevTime = yarp::os::Time::now();

//            count = 0;
//        }

//    }

}

//bool vCornerThread::detectcorner(vQueue patch, int x, int y)
//{

////    std::cout << "central " << x << " " << y << std::endl;

//    //set the final response to be centred on the current event
//    convolution.setResponseCenter(x, y);

//    //update filter response
//    for(unsigned int i = 0; i < patch.size(); i++)
//    {
//        //events the patch
//        auto vi = is_event<AE>(patch[i]);

////        std::cout << vi->x << " " << vi->y << std::endl;

////        std::cout << "Applying sobel filter..." << std::endl;
//        convolution.applysobel(vi);

//    }

////    std::cout << "Applying gaussian filter...";
//    convolution.applygaussian();

////    std::cout << "Getting score...";
//    double score = convolution.getScore();

////    std::cout << score << std::endl;

//    //reset responses
//    convolution.reset();

//    if(debugOut.getOutputCount()) {
//        yarp::os::Bottle &scorebottleout = debugOut.prepare();
//        scorebottleout.clear();
//        scorebottleout.addDouble(score);
//        debugOut.write();
//    }

//    //if score > thresh tag ae as ce
//    return score > thresh;

//}

void vCornerThread::threadRelease()
{
    std::cout << "Releasing thread..." << std::endl;

    for(int i = 0; i < nthreads; i++)
        delete computeThreads[i];

    debugOut.close();
    vBottleOut.close();
    eventhandler.stop();
    std::cout << "Thread Released Successfully" <<std::endl;

}

/*////////////////////////////////////////////////////////////////////////////*/
//threaded computation
/*////////////////////////////////////////////////////////////////////////////*/
vComputeThread::vComputeThread(int sobelsize, int windowRad, double sigma, double thresh)
{
    this->sobelsize = sobelsize;
    this->windowRad = windowRad;
    this->sigma = sigma;
    this->thresh = thresh;
    int gaussiansize = 2*windowRad + 2 - sobelsize;
    convolution.configure(sobelsize, gaussiansize);
    convolution.setSobelFilters();
    convolution.setGaussianFilter(sigma);
}

void vComputeThread::setData(vQueue *patch)
{
    this->patch = patch;
}

void vComputeThread::run()
{
//    std::cout << "running " << std::endl;
    auto aep = is_event<AE>(patch->front());
    if(detectcorner(aep->x, aep->y)) {
        cornerevt = make_event<LabelledAE>(aep);
        cornerevt->ID = 1;
    }

}

bool vComputeThread::detectcorner(int x, int y)
{

    //set the final response to be centred on the current event
    convolution.setResponseCenter(x, y);

    //update filter response
    for(unsigned int i = 0; i < (*patch).size(); i++)
    {
        //events the patch
        auto vi = is_event<AE>((*patch)[i]);
        convolution.applysobel(vi);

    }
    convolution.applygaussian();

    double score = convolution.getScore();

    //reset responses
    convolution.reset();

    //if score > thresh tag ae as ce
    return score > thresh;

}

ev::event<ev::LabelledAE> vComputeThread::getResponse()
{
    return cornerevt;
}
//empty line to make gcc happy
