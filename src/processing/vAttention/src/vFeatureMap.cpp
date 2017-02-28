//
// Created by miacono on 28/02/17.
//

#include "vFeatureMap.h"

vFeatureMap::vFeatureMap(int r, int c, int rPadding, int cPadding) : Matrix(r + rPadding, c + cPadding) {
    this->rPadding = rPadding;
    this->cPadding = cPadding;
}

PointXY::PointXY(int x, int y) : x(x), y(y){}

PointXY::PointXY() {}

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


int Rectangle::getHeight() {
    return abs(topLeftCorner.y - bottomRightCorner.y);
}

int Rectangle::getWidth(){
    return abs(topLeftCorner.x - bottomRightCorner.x);
}

PointXY Rectangle::getTopLeftCorner() {
    return topLeftCorner;
}

PointXY Rectangle::getBottomRightCorner() {
    return bottomRightCorner;
}

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

