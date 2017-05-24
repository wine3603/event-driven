#ifndef __VCOLLECTSEND__
#define __VCOLLECTSEND__

#include <iCub/eventdriven/vCodec.h>
#include <iCub/eventdriven/vBottle.h>
#include <yarp/os/all.h>

namespace ev {

class collectorPort : public yarp::os::RateThread
{
private:

    vBottle filler;
    yarp::os::BufferedPort<vBottle> sendPort;
    yarp::os::Mutex m;
    yarp::os::Stamp ystamp;
    int prevstamp;



public:
    collectorPort() : RateThread(1.0) {

        prevstamp = 0;

    }

    bool open(std::string name) {

        return sendPort.open(name);

    }

    void pushevent(event<> v, yarp::os::Stamp y) {

        m.lock();
//        if((v->stamp - prevstamp) < 0)
//            std::cout << v->stamp << std::endl;
        prevstamp = v->stamp;

        filler.addEvent(v);
        ystamp = y;
        m.unlock();

    }

    void run() {

        if(filler.size()) {
            vBottle &b = sendPort.prepare();

            m.lock();
            b = filler;
            filler.clear();
            sendPort.setEnvelope(ystamp);
            m.unlock();

        }

        sendPort.write();

    }

};



}

#endif
