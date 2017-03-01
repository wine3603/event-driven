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

using namespace yarp::math;

class PointXY{
public:
    PointXY(int x, int y): x(x),
                           y(y) {};
    PointXY(){};

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
    Rectangle(){};

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
    vFeatureMap():Matrix(){};

    vFeatureMap(int r, int c, int rPadding = 0, int cPadding = 0): Matrix(r + 2 * rPadding, c + 2 * cPadding),
                                                                   rPadding(rPadding),
                                                                   cPadding(cPadding) {};

    void updateWithFilter(yarp::sig::Matrix filter, int row, int col,
                          vFeatureMap *outputMap = YARP_NULLPTR, double upBound = 0, double lowBound = 0);

    void convertToImage(yarp::sig::ImageOf<yarp::sig::PixelBgr> image);
    void threshold(double threshold, bool binary = false, vFeatureMap *outputMap = YARP_NULLPTR);
    void normaliseMap(vFeatureMap* outputMap = YARP_NULLPTR);
    void max(int &rowMax, int &colMax);
    double totalEnergy();
    int getRowPadding(){ return rPadding;};
    int getColPadding(){ return cPadding;};
};

template <typename T>
inline
void clamp(T &val, T min, T max) {
    val = std::max (min, val);
    val = std::min (max, val);
}

#endif //ICUB_EVENT_DRIVEN_VFEATUREMAP_H
