/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Chiara Bartolozzi
 * email:  chiara.bartolozzi@iit.it
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

#include "vAttention.h"
#include <iomanip>
#include <fstream>

using namespace yarp::math;
using namespace yarp::sig;
using namespace yarp::os;
using namespace std;

bool vAttentionModule::configure( ResourceFinder &rf ) {
    
    
    /* set parameters */
    int sensorWidth = rf.check( "sensorWidth", Value( 304 ) ).asInt();
    int sensorHeight = rf.check( "sensorHeight", Value( 240 ) ).asInt();
    tau = rf.check( "tau", Value( 5.0 ) ).asDouble();
    boxWidth = rf.check( "boxWidth", Value( 1 ) ).asInt();
    boxHeight = rf.check( "boxHeight", Value( 1 ) ).asInt();
    int filterSize = rf.check( "filterSize", Value( 15 ) ).asInt();
    bool strictness = rf.check( "strict", Value( false ) ).asBool();
    std::string moduleName = rf.check( "name", Value( "vAttention" ) ).asString();
    
    RFModule::setName( moduleName.c_str() );
    
    /* attach a port of the same name as the module (prefixed with a /) to the module
     so that messages received from the port are redirected to the respond method */
    
    std::string handlerPortName = "/";
    handlerPortName += getName();         // use getName() rather than a literal
    
    if ( !handlerPort.open( handlerPortName.c_str() ) ) {
        std::cout << getName() << ": Unable to open port " << handlerPortName << std::endl;
        return false;
    }
    
    int salMapPadding = filterSize / 2 + filterSize % 2;
    int mapWidth = sensorWidth / boxWidth;
    int mapHeight = sensorHeight / boxHeight;
    
    
    bool check = true;
    moduleName = "/" + moduleName;
    check &= outActivationMapPort.open( moduleName + "/activationMap:o" );
    check &= outSalMapLeftPort.open( moduleName + "/salMapLeft:o" );
    check &= outFeatMap0.open( moduleName + "/featMap0:o" );
    check &= outFeatMap45.open( moduleName + "/featMap45:o" );
    check &= outFeatMap90.open( moduleName + "/featMap90:o" );
    check &= outFeatMap135.open( moduleName + "/featMap135:o" );
    
    double sigmaGabor = (double) filterSize / 4.0;
    double fGabor = 1.0 / (double) filterSize;
    double amplGabor = 0.8;
    int step = 45;
    for ( int theta = 0; theta < 180; theta += step ) {
        vFeatureMap featureMap( mapHeight, mapWidth, salMapPadding, salMapPadding );
        vFeatureMap threshMap( mapHeight, mapWidth, salMapPadding, salMapPadding );
        Matrix filterMap;
        std::string portName = moduleName + "/featMap" + std::to_string( theta ) + ":o";
        generateGaborFilter( filterMap, filterSize, amplGabor, fGabor, sigmaGabor, theta );
//        generateOrientedGaussianFilter(filterMap, amplGabor, sigmaGabor, sigmaGabor, theta, filterSize, filterSize/2,
//                                       filterSize/2);
        orientFeatMap.push_back( featureMap );
        threshFeatMap.push_back( threshMap );
        orientedFilters.push_back( filterMap );
    }
    
    eventMap = vFeatureMap( sensorHeight / boxHeight, sensorWidth / boxWidth );
    salMapLeft = vFeatureMap( mapHeight, mapWidth, salMapPadding, salMapPadding );
    salMapRight = vFeatureMap( mapHeight, mapWidth, salMapPadding, salMapPadding );
    activationMap = vFeatureMap( mapHeight, mapWidth, salMapPadding, salMapPadding );
    
    salMapLeft.zero();
    
    // attach to port
    attManager = new vAttentionManager();
    check &= attManager->open(moduleName, strictness);
    return check;
}

bool vAttentionModule::interruptModule() {
    attManager->interrupt();
    outPort.interrupt();
    outSalMapLeftPort.interrupt();
    outActivationMapPort.interrupt();
    RFModule::interruptModule();
    return true;
}

bool vAttentionModule::close() {
    attManager->close();
    outPort.close();
    outSalMapLeftPort.close();
    outActivationMapPort.close();
    delete attManager;
    RFModule::close();
    return true;
}

bool vAttentionModule::updateModule() {
    
    int x, y, xScaled, yScaled;
    eventMap*=0.95;
    ev::vQueue q = attManager->getRecentEvents(0.5);
    for ( ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++ ) {
        auto aep = ev::is_event<ev::AE>(*qi);
        
        if ( !aep ) { continue; }
        //TODO handle left and right salMap
        
        x = aep->x;
        y = aep->y;
        if ( aep->getChannel() == 0 ) {
            xScaled = x / boxWidth;
            yScaled = y / boxHeight;
            eventMap( yScaled, xScaled )+=50;
        } else {
            //TODO
        }
    }
//    eventMap *= 255;
    Rectangle crop(70,70,220,180);
    vFeatureMap croppedMap;
    eventMap.crop(crop, croppedMap);
    ImageOf<PixelBgr > &imageLeft = outSalMapLeftPort.prepare();
    croppedMap.convertToImage(imageLeft);
    outSalMapLeftPort.write();
    return true;
}

double vAttentionModule::getPeriod() {
    return 0.5;
}

void vAttentionModule::drawBoundingBox( ImageOf<PixelBgr> &image, int topRow, int topCol, int bottomRow, int bottomCol ) {
    
    if ( topRow >= 0 && topRow < image.height() ) {
        for ( int i = topCol; i <= bottomCol; ++i ) {
            if ( i >= 0 && i < image.width() ) {
                image( i, topRow ) = PixelBgr( 255, 0, 0 );
            }
        }
    }
    
    if ( bottomRow >= 0 && bottomRow < image.height() ) {
        for ( int i = topCol; i <= bottomCol; ++i ) {
            if ( i >= 0 && i < image.width() ) {
                image( i, bottomRow ) = PixelBgr( 255, 0, 0 );
            }
        }
    }
    
    if ( topCol >= 0 && topCol < image.width() ) {
        for ( int i = topRow; i <= bottomRow; ++i ) {
            if ( i >= 0 && i < image.height() ) {
                image( topCol, i ) = PixelBgr( 255, 0, 0 );
            }
        }
    }
    
    if ( bottomCol >= 0 && bottomCol < image.width() ) {
        for ( int i = topRow; i <= bottomRow; ++i ) {
            if ( i >= 0 && i < image.height() ) {
                image( bottomCol, i ) = PixelBgr( 255, 0, 0 );
            }
        }
    }
    
};

void vAttentionModule::drawBoundingBox( yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, Rectangle ROI ) {
    int topRow = ROI.getTopLeftCorner().getY();
    int topCol = ROI.getTopLeftCorner().getX();
    int bottomRow = ROI.getBottomRightCorner().getY();
    int bottomCol = ROI.getBottomRightCorner().getX();
    drawBoundingBox( image, topRow, topCol, bottomRow, bottomCol );
}

PointXY vAttentionModule::computeAttentionPoint( const vFeatureMap &map ) {
    
    vFeatureMap copyMap(map);
    copyMap.setSubmatrix(ROI, - std::numeric_limits<double>::max());
    PointXY attPoint = copyMap.max();
    activationMap( attPoint.getY(), attPoint.getX() ) += 50;
    return activationMap.max();
}

bool vAttentionModule::respond( const Bottle &command, Bottle &reply ) {
    std::string helpMessage = std::string( getName().c_str() ) +
                              " commands are: \n" +
                              "help \n" +
                              "quit \n";
    reply.clear();
    
    if ( command.get( 0 ).asString() == "quit" ) {
        reply.addString( "quitting" );
        return false;
    } else if ( command.get( 0 ).asString() == "help" ) {
        std::cout << helpMessage;
        reply.addString( "ok" );
    }
    
    return true;
}

void vAttentionModule::generateOrientedGaussianFilter( Matrix &filterMap, double A, double sigmaX, double sigmaY
                                                        , double theta, int gaussianFilterSize, int xCenter
                                                        , int yCenter ) {
    //Resize to desired size
    filterMap.resize( gaussianFilterSize, gaussianFilterSize );
    filterMap.zero();
    
    double theta_rad = theta * M_PI / 180;
    double a = cos( theta_rad );
    double b = sin( theta_rad );
    double c = cos( theta_rad + M_PI_2 );
    double d = sin( theta_rad + M_PI_2 );
    
    //Generate gaussian filter
    for ( int row = 0; row < gaussianFilterSize; row++ ) {
        for ( int col = 0; col < gaussianFilterSize; col++ ) {
            
            double rDist = row - yCenter;
            double cDist = col - xCenter;
            filterMap( row, col ) = A * exp( -( pow( ( a * rDist + b * cDist ), 2 ) / ( 2 * pow( sigmaX, 2 ) )
                                                + pow( ( c * rDist + d * cDist ), 2 ) / ( 2 * pow( sigmaY, 2 ) ) ) );
        }
    }
}

void vAttentionModule::generateGaborFilter( Matrix &filterMap, int gaborFilterSize, double A, double f, double sigma
                                             , double theta ) {
    filterMap.resize( gaborFilterSize, gaborFilterSize );
    filterMap.zero();
    
    double th_r = theta * M_PI / 180.0;
    
    for ( double x = 0; x < gaborFilterSize; ++x ) {
        for ( double y = 0; y < gaborFilterSize; ++y ) {
            
            double x1 = x - (double) gaborFilterSize / 2.0;
            double y1 = y - (double) gaborFilterSize / 2.0;
            
            filterMap( x, y ) = A * exp( -( pow( x1, 2.0 ) + pow( y1, 2.0 ) ) / ( 2 * pow( sigma, 2 ) ) ) *
                                cos( 2.0 * M_PI * f * ( x1 * cos( th_r ) + y1 * sin( th_r ) ) );
        }
    }
}
/*****************vAttentionManager****************/

bool vAttentionManager::open( const std::string moduleName, bool strictness ) {
    if ( strictness ) {
        std::cout << "Setting " << moduleName << " to strict" << std::endl;
        this->setStrict();
    }
    this->useCallback();
    
    vSurf = new ev::temporalSurface (304,240);
    // why is the input port treated differently???? both in open and close
    std::string inPortName = moduleName + "/vBottle:i";
    return BufferedPort<ev::vBottle>::open( inPortName );
}

void vAttentionManager::close() {
    //close ports
    BufferedPort<ev::vBottle>::close();
    
}

void vAttentionManager::interrupt() {
    //pass on the interrupt call to everything needed
    BufferedPort<ev::vBottle>::interrupt();

}

void vAttentionManager::onRead( ev::vBottle &bot ) {
    /* get the event queue in the vBottle bot */
    ev::vQueue q = bot.get<ev::AE>();
    ev::qsort(q);
    
    mutex.wait();
    for ( ev::vQueue::iterator it = q.begin(); it != q.end(); ++it ) {
        auto ae = ev::is_event(*it);
        vSurf->addEvent(ae);
    }
    mutex.post();
}

ev::vQueue vAttentionManager::getEvents() {
    mutex.wait();
    ev::vQueue outQueue = vSurf->getSurf();
    mutex.post();
    return outQueue;
}

ev::vQueue vAttentionManager::getRecentEvents( double sec ) {
    mutex.wait();
    if (vQueue.empty()) {
        mutex.post();
        return vQueue;
    }
    auto lastEvent = ev::is_event<ev::AE>(vQueue.back());
    unsigned long int currStamp = vtsHelper(lastEvent->stamp);
    unsigned long int maxDt = sec / vtsHelper.tstosecs();
    
    ev::vQueue::iterator cit;
    for (cit = vQueue.begin(); cit != vQueue.end(); ++cit){
        auto aep = ev::is_event<ev::AE>(*cit);
        std::cout << vtsHelper(aep->stamp) << std::endl;
//        std::cout << currStamp << " - " << vtsHelper((*cit).get()->stamp) << " = " << currStamp - vtsHelper((*cit).get()->stamp) << std::endl;
    }
    
    ev::vQueue::reverse_iterator it;
    for (it = vQueue.rbegin(); it != vQueue.rend(); ++it){
        unsigned long int dt = currStamp - vtsHelper((*it).get()->stamp);
        if ( dt > maxDt){
            vQueue.erase(it.base());
        } else break;
    }
    for (cit = vQueue.begin(); cit != vQueue.end(); ++cit){
        std::cout << currStamp - vtsHelper((*cit).get()->stamp) << std::endl;
    }
    mutex.post();
    return getEvents();
}


//empty line to make gcc happy
