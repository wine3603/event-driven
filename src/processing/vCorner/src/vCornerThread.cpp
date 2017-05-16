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

    int gaussiansize = 2*windowRad + 2 - sobelsize;
    convolution.setSobelFilters(sobelsize);
    convolution.setGaussianFilter(sigma, gaussiansize);

}

bool vCornerThread::threadInit()
{   

    std::cout << "Initialising thread..." << std::endl;
    std::cout << "Using a " << sobelsize << "x" << sobelsize << " filter ";
    std::cout << "and a " << 2*windowRad + 1 << "x" << 2*windowRad + 1 << " spatial window" << std::endl;

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

//    for(int i = 0; i < nthreads; i ++)
//        computeThreads.push_back(new vComputeThread());

    std::cout << "Thread initialised" << std::endl;
    return true;
}

void vCornerThread::run()
{
    std::cout << "Starting thread..." << std::endl;
    vQueue patch;

//    ev::vBottle &eventsout = vBottleOut.prepare();
//    eventsout.clear();

    ev::vBottle fillerbottle;

    int count = 0;
    while(!isStopping()) {
        yarp::os::Time::delay(0.0001);
        patch = eventhandler.queryROI(channel, eventsInPatch, windowRad);
        if(!patch.size()) {
//            std::cout << "test " << std::endl;
            continue;
        }
        auto aep = is_event<AE>(patch.front());
        bool isc = detectcorner(patch, aep->x, aep->y);

//        for(int i = 0; i < nthreads; i++) {
//            computeThreads[i]->configure();
//            computeThreads[i]->start();
//        }

//        bool isc = true;
        if(isc) {
            auto ce = make_event<LabelledAE>(aep);
            ce->ID = 1;
            fillerbottle.addEvent(ce);
//            eventsout.addEvent(ce);
            count++;
        }

        double deltat = (yarp::os::Time::now() - prevTime);
        if(deltat > 0.01 && fillerbottle.size()) {
//            std::cout << count << " writing " << eventsout.size() << std::endl;

            yarp::os::Stamp yarpstamp = eventhandler.queryYstamp();
            vBottleOut.setEnvelope(yarpstamp);
            ev::vBottle &eventsout = vBottleOut.prepare();
            eventsout.clear();
            eventsout = fillerbottle;
            vBottleOut.write(strict);
            fillerbottle.clear();

            //            eventsout.clear();

            prevTime = yarp::os::Time::now();

            count = 0;
        }

    }

}

bool vCornerThread::detectcorner(vQueue patch, int x, int y)
{

//    std::cout << "central " << x << " " << y << std::endl;

    //set the final response to be centred on the current event
    convolution.setResponseCenter(x, y);

    //update filter response
    for(unsigned int i = 0; i < patch.size(); i++)
    {
        //events the patch
        auto vi = is_event<AE>(patch[i]);

//        std::cout << vi->x << " " << vi->y << std::endl;

//        std::cout << "Applying sobel filter..." << std::endl;
        convolution.applysobel(vi);

    }

//    std::cout << "Applying gaussian filter...";
    convolution.applygaussian();

//    std::cout << "Getting score...";
    double score = convolution.getScore();

//    std::cout << score << std::endl;

    //reset responses
    convolution.reset();

    if(debugOut.getOutputCount()) {
        yarp::os::Bottle &scorebottleout = debugOut.prepare();
        scorebottleout.clear();
        scorebottleout.addDouble(score);
        debugOut.write();
    }

    //if score > thresh tag ae as ce
    return score > thresh;

}

void vCornerThread::threadRelease()
{
    std::cout << "Releasing thread..." << std::endl;
    debugOut.close();
//    vBottleOut.close();
    eventhandler.stop();
    std::cout << "Thread Released Successfully" <<std::endl;

}

/*////////////////////////////////////////////////////////////////////////////*/
//threaded computation
/*////////////////////////////////////////////////////////////////////////////*/
//void vComputeThread::configure()
//{

//}

//void vComputeThread::run()
//{

//}

//empty line to make gcc happy
