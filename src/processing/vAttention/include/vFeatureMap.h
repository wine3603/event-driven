//
// Created by miacono on 28/02/17.
//

#ifndef ICUB_EVENT_DRIVEN_VFEATUREMAP_H
#define ICUB_EVENT_DRIVEN_VFEATUREMAP_H

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/eventdriven/all.h>
#include <iCub/eventdriven/vtsHelper.h>
#include <math.h>
#include <ostream>

class PointXY{
public:
    PointXY(int x, int y);
    PointXY();

    int x;
    int y;
};

inline
std::ostream &operator<<(std::ostream & str, const PointXY &point) {
    str << "( " << std::to_string(point.x) << ", " << std::to_string(point.y) << " )";
    return str;
}

class Rectangle{
public:
    Rectangle(const PointXY &corner1, const PointXY &corner2, bool isTopLeftZero = true);
    Rectangle(const PointXY &topLeftCorner, int width, int height, bool isTopLeftZero = true);
    Rectangle(int topX, int topY, int bottomX, int bottomY, bool isTopLeftZero = true);
    int getHeight();
    int getWidth();
    PointXY getTopLeftCorner();
    PointXY getBottomRightCorner();
    bool contains(PointXY point);
private:
    PointXY topLeftCorner;
    PointXY bottomRightCorner;
    bool isTopLeftZero;
};

class vFeatureMap : public yarp::sig::Matrix{
private:

    int rPadding;
    int cPadding;

public:
    vFeatureMap(int r, int c, int rPadding = 0, int cPadding = 0);

};


#endif //ICUB_EVENT_DRIVEN_VFEATUREMAP_H
