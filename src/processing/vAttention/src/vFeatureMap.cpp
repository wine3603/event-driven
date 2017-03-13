//
// Created by miacono on 28/02/17.
//

#include "vFeatureMap.h"
#include <limits>
/**vFeatureMap Class Implementation */

void vFeatureMap::updateWithFilter(yarp::sig::Matrix filter, int row, int col, vFeatureMap *outputMap, double upBound,
                                   double lowBound) {
    int filterRows = filter.rows();
    int filterCols = filter.cols();

    yAssert(filterRows <= 2 * rPadding);
    yAssert(filterCols <= 2 * cPadding);

    vFeatureMap* updatedMap;
    if (!outputMap){
        updatedMap = this;
    } else {
        *outputMap = *this;
        updatedMap = outputMap;
    }

    int rMap, cMap;
    // ---- increase energy in the location of the event ---- //
    for (int rFil = 0; rFil < filterRows; rFil++) {
        for (int cFil = 0; cFil < filterCols; cFil++) {
            rMap = row + rFil;
            cMap = col + cFil;
            (*updatedMap)(rMap, cMap) += filter(rFil, cFil);

            if (upBound != 0 && lowBound != 0) {
                clamp((*updatedMap)(row + rFil, col + cFil), lowBound, upBound);
            }
        }
    }

}

void vFeatureMap::convertToImage(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image) {

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

void vFeatureMap::threshold(double threshold, bool binary, vFeatureMap *outputMap) {
    vFeatureMap* thresholdedMap;

    if (!outputMap){
        thresholdedMap = this;
    } else {
        *outputMap = *this;
        thresholdedMap = outputMap;
    }

    for (int i = 0; i < thresholdedMap->rows(); ++i) {
        for (int j = 0; j < thresholdedMap->cols(); ++j) {
            double &thVal = (*thresholdedMap)(i, j);
            double val = (*thresholdedMap)(i,j);
            if (val < threshold)
                thVal = 0;
            else if (binary){
                thVal = 1;
            } else {
                thVal = (*thresholdedMap)(i,j);
            }
        }
    }
}

void vFeatureMap::normalise(vFeatureMap *outputMap) {
    vFeatureMap* normalisedMap;
    if (!outputMap){
        normalisedMap = this;
    } else {
        *outputMap = *this;
        normalisedMap = outputMap;
    }

    *normalisedMap /= normalisedMap->totalEnergy();
}

double vFeatureMap::totalEnergy() {
    double totalEnergy;
    for (int r = 0; r < this->rows(); ++r) {
        for (int c = 0; c < this->cols(); ++c) {
            totalEnergy += (*this)(r,c);
        }
    }
    return totalEnergy;
}

void vFeatureMap::max(int &rowMax, int &colMax) {
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

double vFeatureMap::energyInROI(Rectangle ROI) {

    vFeatureMap croppedMap;
    this->crop(ROI, &croppedMap);
    return croppedMap.totalEnergy();
}

void vFeatureMap::crop(Rectangle ROI, vFeatureMap *outputMap) {
    PointXY topLeft = ROI.getTopLeftCorner();
    PointXY botRight = ROI.getBottomRightCorner();
    yAssert(topLeft.x >= 0 && topLeft.x < this->cols());
    yAssert(topLeft.y >= 0 && topLeft.x < this->rows());
    yAssert(botRight.x >= 0 && botRight.x < this->cols());
    yAssert(botRight.y >= 0 && botRight.x < this->rows());

    int topRow, topCol, bottomRow, bottomCol;
    topCol = topLeft.x;
    bottomCol = botRight.x;

    if (!ROI.isTopLeftOrigin()){
        topRow = (this->rows()-1) - topLeft.y;
        bottomRow = (this->rows()-1) - botRight.y;
    } else {
        topRow = topLeft.y;
        bottomRow = botRight.y;
    }

    vFeatureMap* croppedMap;

    if (!outputMap){
        croppedMap = this;
    } else {
        *outputMap = *this;
        croppedMap = outputMap;
    }

    yarp::sig::Matrix submatrix = this->submatrix(topRow, bottomRow, topCol, bottomCol);
    *croppedMap = vFeatureMap(submatrix);
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

bool Rectangle::contains(PointXY point) {
    bool isIn = true;
    isIn &= (point.x >= topLeftCorner.x && point.x <= bottomRightCorner.x);
    if (isTopLeftZero){
        isIn &= (point.y >= topLeftCorner.y && point.y <= bottomRightCorner.y);
    } else {
        isIn &= (point.y <= topLeftCorner.y && point.y >= bottomRightCorner.y);
    }
    return isIn;
}

