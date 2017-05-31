#include "vCornerTrackingThread.h"

using namespace ev;

vCornerTrackingThread::vCornerTrackingThread(unsigned int height, unsigned int width, std::string name, bool strict,
                                             int mindistance, unsigned int trefresh, int minevts)
{
    this->height = height;
    this->width = width;
    this->name = name;
    this->strict = strict;
    this->mindistance = mindistance;
    this->trefresh = trefresh;
    this->minevts = minevts;
    clusterSet = new clusterPool(mindistance, trefresh, minevts);

}

bool vCornerTrackingThread::threadInit()
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

    std::cout << "Thread initialised" << std::endl;
    return true;
}


void vCornerTrackingThread::onStop()
{
    allocatorCallback.close();
    allocatorCallback.releaseDataLock();

    delete clusterSet;
}


void vCornerTrackingThread::run()
{

    while(true) {

        ev::vQueue *q = 0;
        while(!q && !isStopping()) {
            q = allocatorCallback.getNextQ(yarpstamp);
        }
        if(isStopping()) break;

        std::pair <double, double> vel;
        for(ev::vQueue::iterator qi = q->begin(); qi != q->end(); qi++) {

            //current corner event
            auto cep = is_event<LabelledAE>(*qi);
            if(cep->x > width - 1 || cep->y > height - 1) {
                std::cerr << "Pixel Out of Range: " << std::endl;
                continue;
            }

            //update cluster velocity
            vel = clusterSet->update(cep);

            if(vel.first && vel.second) {
                //create new flow event and assign to it the velocity of the current cluster
                auto fe = make_event<FlowEvent>(cep);
                fe->vx = vel.first;
                fe->vy = vel.second;

                outthread.pushevent(fe, yarpstamp);

            }
        }

        allocatorCallback.scrapQ();

    }

}
