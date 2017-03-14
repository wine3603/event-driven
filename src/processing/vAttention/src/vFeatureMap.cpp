//
// Created by miacono on 28/02/17.
//

#include "vFeatureMap.h"
#include <limits>
/**vFeatureMap Class Implementation */

void vFeatureMap::updateWithFilter(yarp::sig::Matrix filter, int row, int col, vFeatureMap &outputMap, double upBound, double lowBound) const {
    int filterRows = filter.rows();
    int filterCols = filter.cols();

    yAssert(filterRows <= 2 * rPadding);
    yAssert(filterCols <= 2 * cPadding);

    int rMap, cMap;
    // ---- increase energy in the location of the event ---- //
    for (int rFil = 0; rFil < filterRows; rFil++) {
        for (int cFil = 0; cFil < filterCols; cFil++) {
            rMap = row + rFil;
            cMap = col + cFil;
            outputMap(rMap, cMap) += filter(rFil, cFil);

            if (upBound != 0 && lowBound != 0) {
                clamp(outputMap(row + rFil, col + cFil), lowBound, upBound);
            }
        }
    }

}

void vFeatureMap::convertToImage(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image) const {

    int imageRows = rows() - 2 * rPadding;
    int imageCols = cols() - 2 * cPadding;
    image.resize(imageCols, imageRows);
    image.setTopIsLowIndex(true);
    image.zero();

    int shiftedR, shiftedC;
    for (int r = 0; r < imageRows; r++) {
        for (int c = 0; c < imageCols; c++) {
            yarp::sig::PixelBgr pixelBgr;

//          Coordinates of saliency map are shifted by size of padding wrt the image
            shiftedR = r + rPadding;
            shiftedC = c + cPadding;

            double pixelValue = (*this)(shiftedR, shiftedC);
            //negative values in blue, positive in green
            if (pixelValue <= 0) {
                pixelBgr.b = std::min(fabs(pixelValue), 255.0);
            } else {
                pixelBgr.g = std::min(fabs(pixelValue), 255.0);
            }

            image(c, r) = pixelBgr;
        }
    }
}

void vFeatureMap::threshold(double thresh, vFeatureMap &outputMap, bool binary) const {

    yAssert(&outputMap);
    if (&outputMap != this){
        outputMap = *this;
    }

    for (int i = 0; i < outputMap.rows(); ++i) {
        for (int j = 0; j < outputMap.cols(); ++j) {
            double &thVal = outputMap(i, j);
            double val = (*this)(i,j);
            if (val < thresh)
                thVal = 0;
            else if (binary){
                thVal = 1;
            } else {
                thVal = val;
            }
        }
    }
}

void vFeatureMap::normalise(vFeatureMap &outputMap) const {
    yAssert(&outputMap);
    if (&outputMap != this){
        outputMap = *this;
    }
    outputMap /= outputMap.totalEnergy();
}

double vFeatureMap::totalEnergy() const{
    double totalEnergy;
    for (int r = 0; r < this->rows(); ++r) {
        for (int c = 0; c < this->cols(); ++c) {
            totalEnergy += (*this)(r,c);
        }
    }
    return totalEnergy;
}

void vFeatureMap::max(int &rowMax, int &colMax) const{
    rowMax = 0;
    colMax = 0;
    double max = - std::numeric_limits<double >::max();

    for (int r = 0; r < this->rows(); r++) {
        for (int c = 0; c < this->cols(); c++) {
            if ( (*this)(r, c) > max) {
                max = (*this)(r, c);
                rowMax = r;
                colMax = c;
            }
        }
    }
}

double vFeatureMap::energyInROI(Rectangle ROI) const{

    vFeatureMap croppedMap;
    crop(ROI, croppedMap);
    return croppedMap.totalEnergy();
}

void vFeatureMap::crop(Rectangle ROI, vFeatureMap &outputMap) const{
    yAssert(&outputMap);
    PointXY topLeft = ROI.getTopLeftCorner();
    PointXY botRight = ROI.getBottomRightCorner();

    yAssert(topLeft.x >= 0 && topLeft.x < cols());
    yAssert(topLeft.y >= 0 && topLeft.y < rows());
    yAssert(botRight.x >= 0 && botRight.x < cols());
    yAssert(botRight.y >= 0 && botRight.y < rows());

    int topRow, topCol, bottomRow, bottomCol;
    topCol = topLeft.x;
    bottomCol = botRight.x;

    if (!ROI.isTopLeftOrigin()){
        topRow = (rows()-1) - topLeft.y;
        bottomRow = (rows()-1) - botRight.y;
    } else {
        topRow = topLeft.y;
        bottomRow = botRight.y;
    }

    yarp::sig::Matrix submatrix = this->submatrix(topRow, bottomRow, topCol, bottomCol);
    outputMap = vFeatureMap(submatrix);
}

void vFeatureMap::decay(double dt, double tau, vFeatureMap &outputMap) const {
    yAssert(&outputMap);
    if (&outputMap != this){
        outputMap = *this;
    }
    outputMap *= exp(-dt/tau);
}

Rectangle vFeatureMap::computeBoundingBox(PointXY start, double threshold, int increase) const{
    //TODO debug! Energy in ROI considers one additional pixel.

    //Define initial ROI as a square around start of size increase
    PointXY topLeft(start.x - increase, start.y - increase, 0, cols() - 1, 0, rows() - 1);
    PointXY botRight(start.x + increase, start.y + increase, 0, cols() - 1, 0, rows() - 1);
    Rectangle ROI (topLeft, botRight);

    //Defining points above, below, to the left and right of ROI
    PointXY top(topLeft.x, topLeft.y - increase , 0, cols() - 1, 0, rows() - 1);
    PointXY bottom( botRight.x, botRight.y + increase, 0, cols() - 1, 0, rows() - 1);
    PointXY left( topLeft.x - increase,topLeft.y, 0, cols() - 1, 0, rows() - 1);
    PointXY right( botRight.x + increase, botRight.y, 0, cols() - 1, 0, rows() - 1);

    //Computing the energy in the four directions
    double energyTop = energyInROI( Rectangle( top, ROI.getTopRightCorner()));
    double energyBottom = energyInROI( Rectangle(ROI.getBottomLeftCorner(), bottom));
    double energyLeft = energyInROI( Rectangle(left , ROI.getBottomLeftCorner()));
    double energyRight = energyInROI( Rectangle( ROI.getTopRightCorner(), right));

    //Variables to compute the energy growth
    double internalEnergy;
    double previousInternalEnergy = energyInROI(ROI);
    double dEnergy;

    //Pointer to the maximum among the energy in the four directions
    double *maxEnergy;

    do {
        //Computing maximum energy
        maxEnergy = &energyTop;
        if (energyBottom > *maxEnergy)
            maxEnergy = &energyBottom;
        if (energyLeft > *maxEnergy)
            maxEnergy = &energyLeft;
        if (energyRight > *maxEnergy)
            maxEnergy = &energyRight;

        //Increase the size of the bounding box in the direction with the maximum energy
        if (maxEnergy == &energyTop){
            topLeft.y -= increase;
            clamp(topLeft.y, 0, rows()-1);
            top = PointXY( topLeft.x,topLeft.y - increase, 0, cols() - 1, 0, rows() - 1);
            energyTop = energyInROI( Rectangle(top, ROI.getTopRightCorner()));
        }
        if (maxEnergy == &energyBottom){
            botRight.y += increase;
            clamp(botRight.y, 0, rows()-1);
            bottom = PointXY( botRight.x, botRight.y + increase, 0, cols() - 1, 0, rows() - 1);
            energyBottom = energyInROI( Rectangle( ROI.getBottomLeftCorner(), bottom));
        }
        if (maxEnergy == &energyLeft){
            topLeft.x -= increase;
            clamp(topLeft.x, 0, cols()-1);
            left = PointXY ( topLeft.x - increase,topLeft.y, 0, cols() - 1, 0, rows() - 1);
            energyInROI( Rectangle(left, ROI.getBottomLeftCorner()));
        }
        if (maxEnergy == &energyRight){
            botRight.x += increase;
            clamp(botRight.x, 0, cols()-1);
            right = PointXY( botRight.x + increase, botRight.y, 0, cols() - 1, 0, rows() - 1);
            energyInROI( Rectangle( ROI.getTopRightCorner(), right));
        }

        //Update ROI and compute the energy growth rate
        ROI = Rectangle(topLeft,botRight);
        internalEnergy = energyInROI(ROI);
        dEnergy = (internalEnergy - previousInternalEnergy) / previousInternalEnergy;
        previousInternalEnergy = internalEnergy;

    } while (dEnergy > threshold);

    return ROI;
    //TODO debug. There is one extra iteration towards the top
}

void vFeatureMap::convolve(yarp::sig::Matrix filter, vFeatureMap &outputMap) const {

    yAssert(&outputMap);
    if (&outputMap != this){
        outputMap = *this;
    }

    for (int mapRow = rPadding; mapRow < rows() - rPadding - 1; ++mapRow) {
        for (int mapCol = cPadding; mapCol < cols() - cPadding - 1; ++mapCol) {

            for (int filterRow = 0; filterRow < filter.rows(); ++filterRow) {
                for (int filterCol = 0; filterCol < filter.cols(); ++filterCol) {
                    outputMap(mapRow,mapCol) += outputMap(mapRow + filterRow - rPadding, mapCol + filterCol - cPadding) * filter(filterRow, filterCol);
                }
            }

        }
    }
}

/**Rectangle Class implementation */

Rectangle::Rectangle(const PointXY &corner1, const PointXY &corner2, bool isTopLeftZero) {
    bool valid = true;
    valid &= (corner1.x >= 0 && corner1.y >= 0);
    valid &= (corner2.x >= 0 && corner2.y >= 0);
    yAssert(valid);

    int topX = std::min(corner1.x, corner2.x);
    int bottomX = std::max(corner2.x, corner2.x);

    int topY, bottomY;
    if (isTopLeftZero){
        topY = std::min(corner1.y, corner2.y);
        bottomY = std::max(corner1.y, corner2.y);
    } else {
        topY = std::max(corner1.y, corner2.y);
        bottomY = std::min(corner1.y, corner2.y);
    }
    this->isTopLeftZero = isTopLeftZero;
    this->topLeftCorner = PointXY(topX,topY);
    this->bottomRightCorner = PointXY(bottomX, bottomY);
    this->bottomLeftCorner = PointXY (topX,bottomY);
    this->topRightCorner = PointXY (bottomX,topY);
}

Rectangle::Rectangle(const PointXY &topLeftCorner, int width, int height, bool isTopLeftZero) {
    yAssert(width > 0 && height >  0);

    int bottomX = topLeftCorner.x + width;
    int bottomY;
    if (isTopLeftZero){
        bottomY = topLeftCorner.y + height;
    } else {
        bottomY = topLeftCorner.y - height;
    }
    *this = Rectangle(topLeftCorner,PointXY(bottomX,bottomY),isTopLeftZero);
}

Rectangle::Rectangle(int topX, int topY, int bottomX, int bottomY, bool isTopLeftZero) : Rectangle(PointXY(topX,topY), PointXY(bottomX, bottomY), isTopLeftZero){}

bool Rectangle::contains(PointXY point) const{
    bool isIn = true;
    isIn &= (point.x >= topLeftCorner.x && point.x <= bottomRightCorner.x);
    if (isTopLeftZero){
        isIn &= (point.y >= topLeftCorner.y && point.y <= bottomRightCorner.y);
    } else {
        isIn &= (point.y <= topLeftCorner.y && point.y >= bottomRightCorner.y);
    }
    return isIn;
}

/** Point Class implementation */
PointXY::PointXY(int x, int y, int xLowBound, int xUpBound, int yLowBound, int yUpBound) {
    clamp(x, xLowBound, xUpBound);
    clamp(y, yLowBound, yUpBound);
    this->x = x;
    this->y = y;
}
