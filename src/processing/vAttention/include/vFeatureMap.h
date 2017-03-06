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

    bool isTopLeftZero;

    int getHeight();
    int getWidth();
    PointXY getTopLeftCorner();
    PointXY getBottomRightCorner();
    bool contains(PointXY point);

private:
    PointXY topLeftCorner;
    PointXY bottomRightCorner;
};

/**
 * Class for handling feature maps.
 * It extends the Matrix class integrating it with methods and helpers
 * to perform some specific operations.
 */
class vFeatureMap : public yarp::sig::Matrix{
private:

    //padding size for row and cols
    int rPadding;
    int cPadding;

public:
    /**
     * Default constructor from superclass
     */
    vFeatureMap():Matrix(){};
    /**
     * Copy constructor. Turns a simple Matrix in a vFeatureMap
     * @param matrix matrix to copy
     */
    vFeatureMap(yarp::sig::Matrix &matrix) : yarp::sig::Matrix(matrix),
                                             rPadding(0),
                                             cPadding(0){};

    /**
     * Copy constructor with padding size specification
     * @param matrix matrix to copy
     * @param rPadding padding size for rows
     * @param cPadding padding size for cols
     */
    vFeatureMap(yarp::sig::Matrix &matrix, int rPadding, int cPadding) : yarp::sig::Matrix(matrix),
                                                                         rPadding(rPadding),
                                                                         cPadding(cPadding){};

    /**
     * Constructor
     * @param r rows number
     * @param c cols number
     * @param rPadding padding size for rows
     * @param cPadding padding size for cols
     */
    vFeatureMap(int r, int c, int rPadding = 0, int cPadding = 0): yarp::sig::Matrix(r + 2 * rPadding, c + 2 * cPadding),
                                                                   rPadding(rPadding),
                                                                   cPadding(cPadding) {};

    /**
     * Crops the map
     * @param ROI rectangle specifying the region to crop
     * @param outputMap optional parameter to output the cropped map. If not set this map is cropped
     */
    void crop(Rectangle ROI, vFeatureMap *outputMap = YARP_NULLPTR);

    /**
     * Applies filter map in a specific location
     * @param filter filter to be applied
     * @param row row coordinate
     * @param col col coordinate
     * @param outputMap optional parameter to output the updated map. If not set this map is updated
     * @param upBound upper bound for single elements in map
     * @param lowBound lower bound for single elements in map
     */
    void updateWithFilter(yarp::sig::Matrix filter, int row, int col,
                          vFeatureMap *outputMap = YARP_NULLPTR, double upBound = 0, double lowBound = 0);

    /**
     * Converts the map in a visualisable image. Positive values become green and negative blue.
     * The brighter the color the higher the value. Values above 255 saturate at maximum brightness
     * @param image output image
     */
    void convertToImage(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image);

    /**
     * Thresholds the map up to a desired value.
     * @param threshold values below this threshold are set to 0
     * @param binary if true values above the threshold are set to 1. false by default
     * @param outputMap optional parameter to output the thresholded map. If not set this map is thresholded
     */
    void threshold(double threshold, bool binary = false, vFeatureMap *outputMap = YARP_NULLPTR);
    void normalise(vFeatureMap *outputMap = YARP_NULLPTR);
    void max(int &rowMax, int &colMax);
    double totalEnergy();
    double energyInROI(Rectangle ROI);
    int getRowPadding(){ return rPadding;};
    int getColPadding(){ return cPadding;};

/** iterator implementation
    class iterator : std::iterator  <std::random_access_iterator_tag, //iterator category
                                     double,                          //type of accessed elements
                                     int,                             //pointer-to-pointer distance type
                                     double*,                         //pointer type
                                     double&                          //reference type
                             >
    {
    public:
        iterator(pointer ptr) : ptr_(ptr) { }
        iterator operator++(int junk) { iterator i = *this; ptr_++; return i; }
        iterator operator++() { ptr_++; return *this; }
        reference operator*() { return *ptr_; }
        pointer operator->() { return ptr_; }
        bool operator==(const iterator& rhs) { return ptr_ == rhs.ptr_; }
        bool operator!=(const iterator& rhs) { return ptr_ != rhs.ptr_; }
    private:
        pointer ptr_;
    };
*/

};

inline
std::ostream &operator<<(std::ostream& str, const vFeatureMap& map){
    for (int i = 0; i < map.rows(); ++i) {
        str << "[ " ;
        for (int j = 0; j < map.cols(); ++j) {
            str << map(i,j) << " ";
        }
        str << "]" << std::endl;
    }
    return str;
};


template <typename T>
inline
void clamp(T &val, T min, T max) {
    val = std::max (min, val);
    val = std::min (max, val);
}

#endif //ICUB_EVENT_DRIVEN_VFEATUREMAP_H
