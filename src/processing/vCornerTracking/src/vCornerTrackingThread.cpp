#include "vCornerTrackingThread.h"

using namespace ev;

vCornerTrackingThread::vCornerTrackingThread(unsigned int height, unsigned int width, std::string name, bool strict,
                                             int mindistance, unsigned int trefresh, int maxsize, int minevts)
{
    this->height = height;
    this->width = width;
    this->name = name;
    this->strict = strict;
    this->mindistance = mindistance;
    this->trefresh = trefresh;
    this->maxsize = maxsize;
    this->minevts = minevts;
    clusterSet = new clusterPool(mindistance, trefresh, maxsize, minevts);

    outfile.open("clustersvel.txt");

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
    outthread.stop();
}

void vCornerTrackingThread::threadRelease()
{
    delete clusterSet;
    outfile.close();
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

            //unwrap timestamp
            unsigned int currt = unwrapper(cep->stamp);

            //update cluster velocity
            vel = clusterSet->update(cep, currt);
//            std::cout << "becomes " << vel.first << " " << vel.second << std::endl;
//            std::cout << std::endl;

            if(vel.first && vel.second) {
                //create new flow event and assign to it the velocity of the current cluster
                auto fe = make_event<FlowEvent>(cep);
//                auto fe = make_event<GaussianAE>(cep);
                fe->vx = vel.first;
                fe->vy = vel.second;

//                fe->sigx = (float)(vel.first * 1000000);
//                fe->sigy = (float)(vel.second * 1000000);
//                fe->sigxy = 0;
//                fe->polarity = 1;

//                std::cout << fe->sigx << " " << fe->sigy << std::endl;

                outfile << unwrapperflow(fe->stamp) * vtsHelper::tsscaler << " " << vel.first * 1000000 << " " << vel.second * 1000000 << std::endl;

                outthread.pushevent(fe, yarpstamp);

            }
        }

        allocatorCallback.scrapQ();

    }

}
