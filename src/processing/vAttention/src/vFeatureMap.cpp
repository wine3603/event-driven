//
// Created by miacono on 28/02/17.
//

#include "vFeatureMap.h"

/**vFeatureMap Class Implementation */

void vFeatureMap::updateWithFilter( yarp::sig::Matrix filter, int row, int col, vFeatureMap &outputMap, double upBound
                                    , double lowBound ) const {
    int filterRows = filter.rows();
    int filterCols = filter.cols();
    
    yAssert( filterRows <= 2 * rPadding );
    yAssert( filterCols <= 2 * cPadding );
    
    int rMap, cMap;
    // ---- increase energy in the location of the event ---- //
    for ( int rFil = 0; rFil < filterRows; rFil++ ) {
        for ( int cFil = 0; cFil < filterCols; cFil++ ) {
            rMap = row + rFil;
            cMap = col + cFil;
            outputMap( rMap, cMap ) += filter( rFil, cFil );
            
            if ( upBound != 0 && lowBound != 0 ) {
                clamp( outputMap( row + rFil, col + cFil ), lowBound, upBound );
            }
        }
    }
    
}

void vFeatureMap::convertToImage( yarp::sig::ImageOf<yarp::sig::PixelBgr> &image ) const {
    
    int imageRows = rows() - 2 * rPadding;
    int imageCols = cols() - 2 * cPadding;
    image.resize( imageCols, imageRows );
    image.setTopIsLowIndex( true );
    image.zero();
    
    int shiftedR, shiftedC;
    for ( int r = 0; r < imageRows; r++ ) {
        for ( int c = 0; c < imageCols; c++ ) {
            yarp::sig::PixelBgr pixelBgr;

//          Coordinates of saliency map are shifted by size of padding wrt the image
            shiftedR = r + rPadding;
            shiftedC = c + cPadding;
            
            double pixelValue = ( *this )( shiftedR, shiftedC );
            //negative values in blue, positive in green
            if ( pixelValue <= 0 ) {
                pixelBgr.b = std::min( fabs( pixelValue ), 255.0 );
            } else {
                pixelBgr.g = std::min( fabs( pixelValue ), 255.0 );
            }
            
            image( c, r ) = pixelBgr;
        }
    }
}

void vFeatureMap::threshold( double thresh, vFeatureMap &outputMap, bool binary ) const {
    
    yAssert( &outputMap );
    //copy this map to outputMap
    if ( &outputMap != this ) {
        outputMap = *this;
    }
    
    for ( int i = 0; i < outputMap.rows(); ++i ) {
        for ( int j = 0; j < outputMap.cols(); ++j ) {
            double &thVal = outputMap( i, j );
            double val = ( *this )( i, j );
            if ( val < thresh ) {
                thVal = 0;
            } else if ( binary ) {
                thVal = 1;
            } else {
                thVal = val;
            }
        }
    }
}

bool vFeatureMap::normalise( vFeatureMap &outputMap ) const {
    yAssert( &outputMap );
    //copy this map to outputMap
    if ( &outputMap != this ) {
        outputMap = *this;
    }
    double totalEnergy = outputMap.totalEnergy();
    if ( totalEnergy != 0 ) {
        outputMap /= totalEnergy;
        return true;
    } else {
        return false;
    }
}

double vFeatureMap::totalEnergy() const {
    double totalEnergy = 0;
    for ( int r = 0; r < this->rows(); ++r ) {
        for ( int c = 0; c < this->cols(); ++c ) {
            totalEnergy += ( *this )( r, c );
        }
    }
    return totalEnergy;
}

void vFeatureMap::max( int &rowMax, int &colMax ) const {
    rowMax = 0;
    colMax = 0;
    double max = -std::numeric_limits<double>::max();
    
    for ( int r = 0; r < this->rows(); r++ ) {
        for ( int c = 0; c < this->cols(); c++ ) {
            if ( ( *this )( r, c ) > max ) {
                max = ( *this )( r, c );
                rowMax = r;
                colMax = c;
            }
        }
    }
}

PointXY vFeatureMap::max() const {
    int r,c;
    max(r,c);
    return PointXY(c,r);
}

double vFeatureMap::energyInROI( Rectangle ROI ) const {
    
    vFeatureMap croppedMap;
    crop( ROI, croppedMap );
    return croppedMap.totalEnergy();
}

void vFeatureMap::crop( Rectangle ROI, vFeatureMap &outputMap ) const {
    yAssert( &outputMap );
    PointXY topLeft = ROI.getTopLeftCorner();
    PointXY botRight = ROI.getBottomRightCorner();
    
    yAssert( topLeft.getX() >= 0 && topLeft.getX() < cols() );
    yAssert( topLeft.getY() >= 0 && topLeft.getY() < rows() );
    yAssert( botRight.getX() >= 0 && botRight.getX() < cols() );
    yAssert( botRight.getY() >= 0 && botRight.getY() < rows() );
    
    int topRow, topCol, bottomRow, bottomCol;
    topCol = topLeft.getX();
    bottomCol = botRight.getX();
    
    if ( !ROI.isTopLeftOrigin() ) {
        topRow = ( rows() - 1 ) - topLeft.getY();
        bottomRow = ( rows() - 1 ) - botRight.getY();
    } else {
        topRow = topLeft.getY();
        bottomRow = botRight.getY();
    }
    
    yarp::sig::Matrix submatrix = this->submatrix( topRow, bottomRow, topCol, bottomCol );
    outputMap = vFeatureMap( submatrix );
}

void vFeatureMap::decay( double dt, double tau, vFeatureMap &outputMap ) const {
    yAssert( &outputMap );
    //copy this map to outputMap
    if ( &outputMap != this ) {
        outputMap = *this;
    }
    outputMap *= exp( -dt / tau );
}

Rectangle vFeatureMap::computeBoundingBox( PointXY start, double threshold, int increase ) const {
    
    Rectangle mapBoundaries = getMapBoundaries();
    
    //Define initial ROI as a square around start of size increase
    PointXY topLeft( start.getX() - increase, start.getY() - increase, mapBoundaries );
    PointXY botRight( start.getX() + increase, start.getY() + increase, mapBoundaries );
    Rectangle ROI( topLeft, botRight, &mapBoundaries );
    
    //Defining points above, below, to the left and right of ROI constrained to be within the map boundaries
    PointXY top1( topLeft.getX(), topLeft.getY() - increase, mapBoundaries );
    PointXY top2( ROI.getTopRightCorner().getX(), ROI.getTopRightCorner().getY() - 1, mapBoundaries );
    PointXY bottom1( botRight.getX(), botRight.getY() + increase, mapBoundaries );
    PointXY bottom2( ROI.getBottomLeftCorner().getX(), ROI.getBottomLeftCorner().getY() + 1, mapBoundaries );
    PointXY left1( topLeft.getX() - increase, topLeft.getY(), mapBoundaries );
    PointXY left2( ROI.getBottomLeftCorner().getX() - 1, ROI.getBottomLeftCorner().getY(), mapBoundaries );
    PointXY right1( botRight.getX() + increase, botRight.getY(), mapBoundaries );
    PointXY right2( ROI.getTopRightCorner().getX() + 1, ROI.getTopRightCorner().getY(), mapBoundaries );
    
    //Defining Rectangles above, below to the left and to the right of ROI using above defined points
    Rectangle top( top1, top2, &mapBoundaries );
    Rectangle bottom( bottom1, bottom2, &mapBoundaries );
    Rectangle right( right1, right2, &mapBoundaries );
    Rectangle left( left1, left2, &mapBoundaries );
    
    //Computing the energy in the four directions
    double energyTop = energyInROI( top );
    double energyBottom = energyInROI( bottom );
    double energyLeft = energyInROI( left );
    double energyRight = energyInROI( right );
    
    //Variables to compute the energy growth
    double internalEnergy;
    double previousInternalEnergy = energyInROI( ROI );
    double dEnergy;
    
    //Pointer to the maximum among the energy in the four directions
    double *maxEnergy;
    
    do {
        //Computing maximum energy
        maxEnergy = &energyTop;
        if ( energyBottom > *maxEnergy ) {
            maxEnergy = &energyBottom;
        }
        if ( energyLeft > *maxEnergy ) {
            maxEnergy = &energyLeft;
        }
        if ( energyRight > *maxEnergy ) {
            maxEnergy = &energyRight;
        }
        
        //compute the energy growth rate
        internalEnergy = previousInternalEnergy + *maxEnergy;
        dEnergy = ( internalEnergy - previousInternalEnergy ) / previousInternalEnergy;
        previousInternalEnergy = internalEnergy;
        if ( dEnergy <= threshold ) {
            break;
        }
        
        //Increase the size of the bounding box in the direction with the maximum energy
        if ( maxEnergy == &energyTop ) {
            topLeft.translate( 0, -increase );
            ROI = Rectangle( topLeft, botRight, &mapBoundaries );
            top.translate( 0, -increase );
            energyTop = energyInROI( top );
        }
        if ( maxEnergy == &energyBottom ) {
            botRight.translate( 0, increase );
            ROI = Rectangle( topLeft, botRight, &mapBoundaries );
            bottom.translate( 0, increase );
            energyBottom = energyInROI( bottom );
        }
        if ( maxEnergy == &energyLeft ) {
            topLeft.translate( -increase, 0 );
            ROI = Rectangle( topLeft, botRight, &mapBoundaries );
            left.translate( -increase, 0 );
            energyLeft = energyInROI( left );
        }
        if ( maxEnergy == &energyRight ) {
            botRight.translate( increase, 0 );
            ROI = Rectangle( topLeft, botRight, &mapBoundaries );
            right.translate( increase, 0 );
            energyRight = energyInROI( right );
        }
        
        
    } while ( dEnergy > threshold );
    
    return ROI;
}

void vFeatureMap::convolve( yarp::sig::Matrix filter, vFeatureMap &outputMap ) const {
    
    yAssert( &outputMap );
    //copy this map to outputMap
    if ( &outputMap != this ) {
        outputMap = *this;
    }
    
    for ( int mapRow = rPadding; mapRow < rows() - rPadding - 1; ++mapRow ) {
        for ( int mapCol = cPadding; mapCol < cols() - cPadding - 1; ++mapCol ) {
            
            for ( int filterRow = 0; filterRow < filter.rows(); ++filterRow ) {
                for ( int filterCol = 0; filterCol < filter.cols(); ++filterCol ) {
                    outputMap( mapRow, mapCol ) +=
                            outputMap( mapRow + filterRow - rPadding, mapCol + filterCol - cPadding ) *
                            filter( filterRow, filterCol );
                }
            }
            
        }
    }
}

std::ostream &operator<<( std::ostream &str, const vFeatureMap &map ) {
    for ( int i = 0; i < map.rows(); ++i ) {
        str << "[ ";
        for ( int j = 0; j < map.cols(); ++j ) {
            str << map( i, j ) << " ";
        }
        str << "]" << std::endl;
    }
    return str;
}

void vFeatureMap::multiplySubmatrix( Rectangle rec, double value ) {
    PointXY tl = rec.isTopLeftOrigin()?rec.getTopLeftCorner():rec.getBottomLeftCorner();
    PointXY br = rec.isTopLeftOrigin()?rec.getBottomRightCorner():rec.getTopRightCorner();
    
    for ( int r = tl.getY(); r < br.getY(); ++r ) {
        for ( int c = tl.getX(); c < br.getX(); ++c ) {
            (*this)(r,c) *= value;
        }
    }
}


void vFeatureMap::setSubmatrix( Rectangle rec, double value ) {
    PointXY tl = rec.isTopLeftOrigin()?rec.getTopLeftCorner():rec.getBottomLeftCorner();
    PointXY br = rec.isTopLeftOrigin()?rec.getBottomRightCorner():rec.getTopRightCorner();
    
    for ( int r = tl.getY(); r < br.getY(); ++r ) {
        for ( int c = tl.getX(); c < br.getX(); ++c ) {
            (*this)(r,c) = value;
        }
    }
}

/**Rectangle Class implementation */

Rectangle::Rectangle( const PointXY &corner1, const PointXY &corner2, Rectangle *boundary, bool isTopLeftZero ) {
    
    int topX = std::min( corner1.getX(), corner2.getX() );
    int bottomX = std::max( corner1.getX(), corner2.getX() );
    
    int topY, bottomY;
    if ( isTopLeftZero ) {
        topY = std::min( corner1.getY(), corner2.getY() );
        bottomY = std::max( corner1.getY(), corner2.getY() );
    } else {
        topY = std::max( corner1.getY(), corner2.getY() );
        bottomY = std::min( corner1.getY(), corner2.getY() );
    }
    this->isTopLeftZero = isTopLeftZero;
    this->topLeftCorner = PointXY( topX, topY );
    this->bottomRightCorner = PointXY( bottomX, bottomY );
    this->bottomLeftCorner = PointXY( topX, bottomY );
    this->topRightCorner = PointXY( bottomX, topY );
    if ( boundary ) {
        setBoundaries( *boundary );
    }
}

Rectangle::Rectangle( const PointXY &topLeftCorner, int width, int height, Rectangle *boundary, bool isTopLeftZero ) {
    yAssert( width > 0 && height > 0 );
    
    int bottomX = topLeftCorner.getX() + width;
    int bottomY;
    if ( isTopLeftZero ) {
        bottomY = topLeftCorner.getY() + height;
    } else {
        bottomY = topLeftCorner.getY() - height;
    }
    *this = Rectangle( topLeftCorner, PointXY( bottomX, bottomY ), boundary, isTopLeftZero );
}

Rectangle::Rectangle( int topX, int topY, int bottomX, int bottomY, Rectangle *boundary, bool isTopLeftZero )
        : Rectangle(
        PointXY( topX, topY ), PointXY( bottomX, bottomY ), boundary, isTopLeftZero ) {}

bool Rectangle::contains( PointXY point ) const {
    bool isIn = true;
    isIn &= ( point.getX() >= topLeftCorner.getX() && point.getX() <= bottomRightCorner.getX() );
    if ( isTopLeftZero ) {
        isIn &= ( point.getY() >= topLeftCorner.getY() && point.getY() <= bottomRightCorner.getY() );
    } else {
        isIn &= ( point.getY() <= topLeftCorner.getY() && point.getY() >= bottomRightCorner.getY() );
    }
    return isIn;
}

std::ostream &operator<<( std::ostream &str, const Rectangle &rectangle ) {
    
    str << "\n" << rectangle.getTopLeftCorner() << "-----------" <<
        rectangle.getTopRightCorner()
        << "\n    |                    |\n    |                    |\n    |                    |\n    |                    |\n    |                    |\n"
        <<
        rectangle.getBottomLeftCorner() << "-----------" <<
        rectangle.getBottomRightCorner() << "\n";
    return str;
}

void Rectangle::translate( int dx, int dy ) {
    topLeftCorner.translate( dx, dy );
    topRightCorner.translate( dx, dy );
    bottomLeftCorner.translate( dx, dy );
    bottomRightCorner.translate( dx, dy );
}

void Rectangle::setBoundaries( Rectangle &boundary ) {
    topLeftCorner.setBoundaries( boundary );
    topRightCorner.setBoundaries( boundary );
    bottomRightCorner.setBoundaries( boundary );
    bottomLeftCorner.setBoundaries( boundary );
    bounded = true;
}

/** Point Class implementation */
PointXY::PointXY( int x, int y, int xLowBound, int xUpBound, int yLowBound, int yUpBound ) {
    clamp( x, xLowBound, xUpBound );
    clamp( y, yLowBound, yUpBound );
    this->x = x;
    this->y = y;
    this->xLowBound = xLowBound;
    this->xUpBound = xUpBound;
    this->yLowBound = yLowBound;
    this->yUpBound = yUpBound;
    this->bounded = true;
}

PointXY::PointXY( int x, int y, Rectangle &boundary ) {
    *this = PointXY( x, y, true );
    setBoundaries( boundary );
}

void PointXY::translate( int dx, int dy ) {
    x += dx;
    y += dy;
    if ( bounded ) {
        clamp( x, xLowBound, xUpBound );
        clamp( y, yLowBound, yUpBound );
    }
}

std::ostream &operator<<( std::ostream &str, const PointXY &point ) {
    str << "( " << std::to_string( point.getX() ) << ", " << std::to_string( point.y ) << " )";
    return str;
}

void PointXY::setBoundaries( Rectangle boundary ) {
    if ( boundary.isTopLeftOrigin() ) {
        yLowBound = boundary.getTopLeftCorner().y;
        yUpBound = boundary.getBottomLeftCorner().y;
    } else {
        yLowBound = boundary.getBottomLeftCorner().y;
        yUpBound = boundary.getTopLeftCorner().y;
    }
    xLowBound = boundary.getTopLeftCorner().x;
    xUpBound = boundary.getBottomRightCorner().x;
    clamp( x, xLowBound, xUpBound );
    clamp( y, yLowBound, yUpBound );
    bounded = true;
}

Rectangle PointXY::getBoundaries() const {
    return Rectangle( xLowBound, yLowBound, xUpBound, yUpBound );
}

