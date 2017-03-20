//
// Created by miacono on 28/02/17.
//

#ifndef ICUB_EVENT_DRIVEN_VFEATUREMAP_H
#define ICUB_EVENT_DRIVEN_VFEATUREMAP_H

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/math/Math.h>
#include <iCub/eventdriven/all.h>
#include <iCub/eventdriven/vtsHelper.h>
#include <math.h>
#include <ostream>
#include <limits>

using namespace yarp::math;

class Rectangle;

class PointXY;

class vFeatureMap;

/**
 * Class defining a 2D point
 */
class PointXY {
public:
    PointXY( int x, int y, bool bounded = false ) : x( x ),
                                                    y( y ),
                                                    bounded( bounded ) {};
    
    PointXY( int x, int y, int xLowBound, int xUpBound, int yLowBound, int yUpBound );
    
    PointXY( int x, int y, Rectangle &boundary );
    
    PointXY() : x( 0 ),
                y( 0 ),
                bounded( false ) {};
    
    void translate( int dx, int dy );
    
    void setBoundaries( Rectangle boundary );
    
    Rectangle getBoundaries() const;
    
    bool isBounded() const { return bounded; };
    
    int getX() const { return x; }
    
    int getY() const { return y; }
    
    friend std::ostream &operator<<( std::ostream &str, const PointXY &point );


private:
    
    int x;
    int y;
    bool bounded;
    int xLowBound;
    int xUpBound;
    int yLowBound;
    int yUpBound;
    
};

/**
 * Class for defining rectangular bounding boxes or ROI.
 */
class Rectangle {
public:
    /**
     * Constructor. Takes as argument two PointXY defining two opposite corners
     * @param corner1
     * @param corner2
     * @param isTopLeftZero If true the origin of the coordinate reference system is at the top left, otherwise at bottom left.
     */
    Rectangle( const PointXY &corner1, const PointXY &corner2, Rectangle *boundary = YARP_NULLPTR
               , bool isTopLeftZero = true );
    
    /**
     * Constructor. Takes the coordinate of top left corner and the desired rectangle width and height
     * @param topLeftCorner
     * @param width
     * @param height
     * @param isTopLeftZero If true the origin of the coordinate reference system is at the top left, otherwise at bottom left.
     */
    Rectangle( const PointXY &topLeftCorner, int width, int height, Rectangle *boundary = YARP_NULLPTR
               , bool isTopLeftZero = true );
    
    /**
     * Constructor. Takes the coordinates of two opposite corners
     * @param topX
     * @param topY
     * @param bottomX
     * @param bottomY
     * @param isTopLeftZero If true the origin of the coordinate reference system is at the top left, otherwise at bottom left.
     */
    Rectangle( int topX, int topY, int bottomX, int bottomY, Rectangle *boundary = YARP_NULLPTR
               , bool isTopLeftZero = true );
    
    /**
     * Default constructor
     */
    Rectangle() {};
    
    /**
     * Tells whether a given point belongs to the rectangle region
     * @param point
     * @return true if point is contained in the rectangle
     */
    bool contains( PointXY point ) const;
    
    void translate( int dx, int dy );
    
    int getHeight() const { return abs( topLeftCorner.getY() - bottomRightCorner.getY() ); };
    
    int getWidth() const { return abs( topLeftCorner.getX() - bottomRightCorner.getX() ); };
    
    PointXY getTopLeftCorner() const { return topLeftCorner; };
    
    PointXY getBottomRightCorner() const { return bottomRightCorner; };
    
    PointXY getTopRightCorner() const { return topRightCorner; }
    
    PointXY getBottomLeftCorner() const { return bottomLeftCorner; }
    
    bool isTopLeftOrigin() const { return isTopLeftZero; };
    
    void setBoundaries( Rectangle &boundary );
    
    friend std::ostream &operator<<( std::ostream &str, const Rectangle &rectangle );

private:
    bool bounded;
    PointXY topLeftCorner;
    PointXY topRightCorner;
    PointXY bottomLeftCorner;
    PointXY bottomRightCorner;
    bool isTopLeftZero;
    
};


/**
 * Class for handling feature maps.
 * It extends the Matrix class integrating it with methods and helpers
 * to perform some specific operations.
 */
class vFeatureMap : public yarp::sig::Matrix {
private:
    int
    //padding size for row and cols
    int rPadding;
    int cPadding;

public:
    /**
     * Default constructor from superclass
     */
    vFeatureMap() : Matrix() {};
    
    /**
     * Copy constructor. Turns a simple Matrix in a vFeatureMap
     * @param matrix matrix to copy
     */
    vFeatureMap( yarp::sig::Matrix &matrix ) : yarp::sig::Matrix( matrix ),
                                               rPadding( 0 ),
                                               cPadding( 0 ) {}
    
    /**
     * Copy constructor with padding size specification
     * @param matrix matrix to copy
     * @param rPadding padding size for rows
     * @param cPadding padding size for cols
     */
    vFeatureMap( yarp::sig::Matrix &matrix, int rPadding, int cPadding ) : yarp::sig::Matrix( matrix ),
                                                                           rPadding( rPadding ),
                                                                           cPadding( cPadding ) {}
    
    /**
     * Constructor
     * @param r rows number
     * @param c cols number
     * @param rPadding padding size for rows
     * @param cPadding padding size for cols
     */
    vFeatureMap( int r, int c, int rPadding = 0, int cPadding = 0 ) : yarp::sig::Matrix( r + 2 * rPadding,
                                                                                         c + 2 * cPadding ),
                                                                      rPadding( rPadding ),
                                                                      cPadding( cPadding ) {}
    
    /**
     * Crops the map
     * @param ROI rectangle specifying the region to crop
     * @param outputMap optional parameter to output the cropped map. If not set this map is cropped
     */
    void crop( Rectangle ROI, vFeatureMap &outputMap ) const;
    
    void crop( Rectangle ROI ) { crop( ROI, *this ); }
    
    /**
     * Applies filter map in a specific location
     * @param filter filter to be applied
     * @param row row coordinate
     * @param col col coordinate
     * @param outputMap optional parameter to output the updated map. If not set this map is updated
     * @param upBound upper bound for single elements in map
     * @param lowBound lower bound for single elements in map
     */
    void updateWithFilter( yarp::sig::Matrix filter, int row, int col, vFeatureMap &outputMap, double upBound = 0
                           , double lowBound = 0 ) const;
    
    void updateWithFilter( yarp::sig::Matrix filter, int row, int col, double upBound = 0, double lowBound = 0 ) {
        updateWithFilter( filter, row, col, *this, upBound, lowBound );
    }
    
    
    /**
     * Performs convolution on outputmap using the passed filter
     * @param filter
     * @param outputMap optional parameter to convolves the updated map. If not set this map is convolved
     */
    void convolve( yarp::sig::Matrix filter, vFeatureMap &outputMap ) const;
    
    void convolve( yarp::sig::Matrix filter ) { convolve( filter, *this ); };
    
    
    /**
     * Converts the map in a visualisable image. Positive values become green and negative blue.
     * The brighter the color the higher the value. Values above 255 saturate at maximum brightness
     * @param image output image
     */
    void convertToImage( yarp::sig::ImageOf<yarp::sig::PixelBgr> &image ) const;
    
    /**
     * Thresholds the map up to a desired value.
     * @param thresh values below this threshold are set to 0
     * @param binary if true values above the threshold are set to 1. false by default
     * @param outputMap optional parameter to output the thresholded map. If not set this map is thresholded
     */
    void threshold( double thresh, vFeatureMap &outputMap, bool binary = false ) const;
    
    void threshold( double thresh, bool binary = false ) { threshold( thresh, *this, binary ); }
    
    /**
     * Normalises the map so that its energy sums to 1.
     * @param outputMap optional parameter to output the normalised map. If not set this map is normalised
     */
    bool normalise( vFeatureMap &outputMap ) const;
    
    void normalise() { normalise( *this ); }
    
    /**
     * Finds the maximum in the map and outputs its coordinates
     * @param rowMax output row of max value
     * @param colMax output col of max value
     */
    void max( int &rowMax, int &colMax ) const;
    
    /**
     * Exponentially decays the map. Decay fator = e^(-dt/tau).
     * @param dt
     * @param tau
     * @param outputMap outputMap optional parameter to output the decayed map. If not set this map is decayed
     */
    void decay( double dt, double tau, vFeatureMap &outputMap ) const;
    
    void decay( double dt, double tau ) { decay( dt, tau, *this ); }
    
    /**
     * Computes the sum of the energy of every element in the map
     * @return total energy
     */
    double totalEnergy() const;
    
    /**
     * Computes the sum of the energy of the elements of a certain ROI in the map
     * @param ROI
     * @return energy in ROI
     */
    double energyInROI( Rectangle ROI ) const;
    
    /**
     * Heuristic to compute a bounding box around a certain point.
     * The algorithm increases the size of the box in one of the four directions (up, down, left, right) which
     * gives the higher contribute in terms of energy contained inside the box. The search stops when the energy growth rate
     * drops below the threshold passed as argument.
     * @param start Point at which the algorithm starts
     * @param threshold minimum growth rate that stops the search.
     * @param increase box size increases by this value in one direction at every step
     * @return the computed bounding box
     */
    Rectangle computeBoundingBox( PointXY start, double threshold, int increase ) const;
    
    /**
     * Returns a Rectangle containing all the points of the map
     * @return map bounding box
     */
    Rectangle getMapBoundaries() const { return Rectangle( 0, 0, cols() - 1, rows() - 1 ); }
    
    int getRowPadding() { return rPadding; }
    
    int getColPadding() { return cPadding; }
    
    friend std::ostream &operator<<( std::ostream &str, const vFeatureMap &map );
    
};


template< typename T >
inline
void clamp( T &val, T min, T max ) {
    val = std::max( min, val );
    val = std::min( max, val );
}

#endif //ICUB_EVENT_DRIVEN_VFEATUREMAP_H
