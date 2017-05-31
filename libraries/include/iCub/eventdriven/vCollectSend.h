#ifndef __VCOLLECTSEND__
#define __VCOLLECTSEND__

#include <iCub/eventdriven/vCodec.h>
#include <iCub/eventdriven/vBottle.h>
#include <iCub/eventdriven/vtsHelper.h>
#include <yarp/os/all.h>
#include <limits>
#include <iomanip>

typedef std::numeric_limits< double > dbl;

namespace ev {

class collectorPort : public yarp::os::RateThread
{
private:

    vBottle filler;
    yarp::os::BufferedPort<vBottle> sendPort;
    yarp::os::Mutex m;
    yarp::os::Stamp ystamp;
//    double currystamp;
//    double prevystamp;
//    int prevstamp;



public:
    collectorPort() : RateThread(1.0) {

//        prevstamp = 0;
//        prevystamp = 0.0;

    }

    bool open(std::string name) {

        return sendPort.open(name);

    }

    void pushevent(event<> v, yarp::os::Stamp y) {

        m.lock();
//        if((v->stamp - prevstamp) < 0)
//            std::cout << prevstamp * vtsHelper::tsscaler << " " << v->stamp * vtsHelper::tsscaler << std::endl;
//        prevstamp = v->stamp;

        filler.addEvent(v);
        ystamp = y;
//        currystamp = ystamp.getTime();
        m.unlock();

    }

    void run() {

        if(filler.size()) {
            vBottle &b = sendPort.prepare();

            m.lock();
            b = filler;
            filler.clear();
            sendPort.setEnvelope(ystamp);
//            std::cout.precision( dbl::digits10 );
//            std::cout << (currystamp - prevystamp) << std::endl;
//            prevystamp = currystamp;
            m.unlock();

            sendPort.write();

        }

//         if(!(currystamp - prevystamp))
//             sendPort.write();

    }

};



}

#endif
