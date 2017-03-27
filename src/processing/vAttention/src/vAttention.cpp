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
    //set the name of the module
    std::string moduleName =
            rf.check( "name", Value( "vAttention" ) ).asString();
    RFModule::setName( moduleName.c_str() );
    
    /* attach a port of the same name as the module (prefixed with a /) to the module
     so that messages received from the port are redirected to the respond method */
    
    std::string handlerPortName = "/";
    handlerPortName += getName();         // use getName() rather than a literal
    
    if ( !handlerPort.open( handlerPortName.c_str() ) ) {
        std::cout << getName() << ": Unable to open port " << handlerPortName << std::endl;
        return false;
    }
    
    attach( handlerPort );                  // attach to port
    attManager = new vAttentionManager();
    return attManager->initialize( rf );
    
}

bool vAttentionModule::interruptModule() {
    attManager->interrupt();
    RFModule::interruptModule();
    return true;
}

bool vAttentionModule::close() {
    attManager->close();
    delete attManager;
    RFModule::close();
    return true;
}

bool vAttentionModule::updateModule() {
    return true;
}

double vAttentionModule::getPeriod() {
    return 1;
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

/*****************vAttentionManager****************/

void vAttentionManager::generateOrientedGaussianFilter( Matrix &filterMap, double A, double sigmaX, double sigmaY
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

void vAttentionManager::generateGaborFilter( Matrix &filterMap, int gaborFilterSize, double A, double f, double sigma
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

bool vAttentionManager::open( const std::string moduleName, bool strictness ) {
    if ( strictness ) {
        std::cout << "Setting " << moduleName << " to strict" << std::endl;
        this->setStrict();
    }
    this->useCallback();
    
    // why is the input port treated differently???? both in open and close
    std::string inPortName = moduleName + "/vBottle:i";
    return BufferedPort<ev::vBottle>::open( inPortName );
}

void vAttentionManager::close() {
    //close ports
    outPort.close();
    BufferedPort<ev::vBottle>::close();
    outSalMapLeftPort.close();
    outActivationMapPort.close();
}

void vAttentionManager::interrupt() {
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    BufferedPort<ev::vBottle>::interrupt();
    outSalMapLeftPort.interrupt();
    outActivationMapPort.interrupt();
}

void vAttentionManager::onRead( ev::vBottle &bot ) {
    /* get the event queue in the vBottle bot */
    ev::vQueue q = bot.get<ev::AddressEvent>();
    
    q.sort( true );
    
    int x, y, xScaled, yScaled;
    eventMap.zero();
    for ( ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++ ) {
        ev::event<ev::AddressEvent> aep = ev::getas<ev::AddressEvent>( *qi );
        
        if ( !aep ) { continue; }
        //TODO handle left and right salMap
        
        x = aep->getX();
        y = aep->getY();
        if ( aep->getChannel() == 0 ) {
            xScaled = x / boxWidth;
            yScaled = y / boxHeight;
            eventMap( yScaled, xScaled )++;
        } else {
            //TODO
        }
    }
    
    double ampl;
    
    double upBound = 2000, lowBound = -2000;
    for ( int r = 0; r < eventMap.rows(); ++r ) {
        for ( int c = 0; c < eventMap.cols(); ++c ) {
            ampl = (double) eventMap( r, c );
            if ( ampl != 0 ) {
                for ( unsigned int i = 0; i < orientFeatMap.size(); ++i ) {
                    orientFeatMap[i].updateWithFilter( orientedFilters[i], r, c, upBound, lowBound );
                }
            }
        }
    }
    
    unsigned long int t = unwrap( q.back().get()->getStamp() );
    
    double dt = (double) ( t - prevT ) * ( 80 * pow( 10, -9 ) );
    prevT = t;
    double th = 50;
    bool binary = false;
    std::vector<ImageOf<PixelBgr> > images( 4 );
    
    images[0] = outFeatMap0.prepare();
    images[1] = outFeatMap45.prepare();
    images[2] = outFeatMap90.prepare();
    images[3] = outFeatMap135.prepare();
    
    for ( unsigned int j = 0; j < orientFeatMap.size(); ++j ) {
        orientFeatMap[j].decay( dt, tau );
        orientFeatMap[j].threshold( th, threshFeatMap[j], binary );
        threshFeatMap[j].normalise();
        salMapLeft += threshFeatMap[j];
        threshFeatMap[j] *= 200000;
        threshFeatMap[j].convertToImage( images[j] );
    }
    
    activationMap.decay( dt, tau );
    
    salMapLeft.normalise();
    int r, c;
    computeAttentionPoint( salMapLeft, r, c );
    PointXY attPoint( c, r );
    
    Rectangle ROI = salMapLeft.computeBoundingBox( attPoint, 0.01, 5 );
    salMapLeft *= 200000;
    
    
    //  --- convert to images for display --- //
    ImageOf<PixelBgr> &imageLeft = outSalMapLeftPort.prepare();
    ImageOf<PixelBgr> &imageActivation = outActivationMapPort.prepare();
    
    activationMap.convertToImage( imageActivation );
    
    salMapLeft.convertToImage( imageLeft );
    ROI.translate( -salMapLeft.getColPadding(), -salMapLeft.getRowPadding() );
    attPoint.setBoundaries( Rectangle( PointXY( 0, 0 ), PointXY( imageLeft.width(), imageLeft.height() ) ) );
    attPoint.translate( -salMapLeft.getColPadding(), -salMapLeft.getRowPadding() );
    imageLeft( attPoint.getX(), attPoint.getY() ) = PixelBgr( 255, 0, 0 );
    drawBoundingBox( imageLeft, ROI );
    
    outSalMapLeftPort.write();
    outActivationMapPort.write();
    outFeatMap0.write();
    outFeatMap45.write();
    outFeatMap90.write();
    outFeatMap135.write();
}

void vAttentionManager::computeAttentionPoint( const vFeatureMap &map, int &attPointRow, int &attPointCol ) {
    
    map.max( attPointRow, attPointCol );
    activationMap( attPointRow, attPointCol ) += 20;
    activationMap.max( attPointRow, attPointCol );
    
}

void
vAttentionManager::drawBoundingBox( ImageOf<PixelBgr> &image, int topRow, int topCol, int bottomRow, int bottomCol ) {
    
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

//    return true;
};

void vAttentionManager::drawBoundingBox( yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, Rectangle ROI ) {
    int topRow = ROI.getTopLeftCorner().getY();
    int topCol = ROI.getTopLeftCorner().getX();
    int bottomRow = ROI.getBottomRightCorner().getY();
    int bottomCol = ROI.getBottomRightCorner().getX();
    drawBoundingBox( image, topRow, topCol, bottomRow, bottomCol );
}


bool vAttentionManager::initialize( ResourceFinder &rf ) {
    
    /* set parameters */
    int sensorWidth = rf.check( "sensorWidth", Value( 304 ) ).asInt();
    int sensorHeight = rf.check( "sensorHeight", Value( 240 ) ).asInt();
    tau = rf.check( "tau", Value( 3.0 ) ).asDouble();
    boxWidth = rf.check( "boxWidth", Value( 2 ) ).asInt();
    boxHeight = rf.check( "boxHeight", Value( 2 ) ).asInt();
    int filterSize = rf.check( "filterSize", Value( 15 ) ).asInt();
    bool strictness = rf.check( "strict", Value( false ) ).asBool();
    std::string moduleName = rf.check( "name", Value( "vAttention" ) ).asString();
    
    
    int salMapPadding = filterSize / 2 + filterSize % 2;
    int mapWidth = sensorWidth / boxWidth;
    int mapHeight = sensorHeight / boxHeight;
    
    
    bool check = true;
    moduleName = "/" + moduleName;
    check &= open( moduleName, strictness );
    check &= outActivationMapPort.open( moduleName + "/activationMap:o" );
    check &= outSalMapLeftPort.open( moduleName + "/salMapLeft:o" );
    check &= outFeatMap0.open( moduleName + "/featMap0:o" );
    check &= outFeatMap45.open( moduleName + "/featMap45:o" );
    check &= outFeatMap90.open( moduleName + "/featMap90:o" );
    check &= outFeatMap135.open( moduleName + "/featMap135:o" );
    
    double sigmaGabor = (double) filterSize / 4.0;
    double fGabor = 2.0 / (double) filterSize;
    double amplGabor = 0.4;
    int step = 45;
    for ( int theta = 0; theta < 180; theta += step ) {
        vFeatureMap featureMap( mapHeight, mapWidth, salMapPadding, salMapPadding );
        vFeatureMap threshMap( mapHeight, mapWidth, salMapPadding, salMapPadding );
        Matrix filterMap;
        std::string portName = moduleName + "/featMap" + std::to_string( theta ) + ":o";
//        generateGaborFilter( filterMap, filterSize, amplGabor, fGabor, sigmaGabor, theta );
        generateOrientedGaussianFilter(filterMap, amplGabor, sigmaGabor, sigmaGabor, theta, filterSize, filterSize/2,
                                       filterSize/2);
        orientFeatMap.push_back( featureMap );
        threshFeatMap.push_back( threshMap );
        orientedFilters.push_back( filterMap );
    }
    
    eventMap = vFeatureMap( sensorHeight / boxHeight, sensorWidth / boxWidth );
    salMapLeft = vFeatureMap( mapHeight, mapWidth, salMapPadding, salMapPadding );
    salMapRight = vFeatureMap( mapHeight, mapWidth, salMapPadding, salMapPadding );
    activationMap = vFeatureMap( mapHeight, mapWidth, salMapPadding, salMapPadding );
    
    return check;
}


//empty line to make gcc happy
