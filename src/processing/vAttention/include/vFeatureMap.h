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

/**
 * Class defining a 2D point
 */
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

/**
 * Class for defining rectangular bounding boxes or ROI.
 */
class Rectangle{
public:
    /**
     * Constructor. Takes as argument two PointXY defining two opposite corners
     * @param corner1
     * @param corner2
     * @param isTopLeftZero If true the origin of the coordinate reference system is at the top left, otherwise at bottom left.
     */
    Rectangle(const PointXY &corner1, const PointXY &corner2, bool isTopLeftZero = true);
    /**
     * Constructor. Takes the coordinate of top left corner and the desired rectangle width and height
     * @param topLeftCorner
     * @param width
     * @param height
     * @param isTopLeftZero If true the origin of the coordinate reference system is at the top left, otherwise at bottom left.
     */
    Rectangle(const PointXY &topLeftCorner, int width, int height, bool isTopLeftZero = true);
    /**
     * Constructor. Takes the coordinates of two opposite corners
     * @param topX
     * @param topY
     * @param bottomX
     * @param bottomY
     * @param isTopLeftZero If true the origin of the coordinate reference system is at the top left, otherwise at bottom left.
     */
    Rectangle(int topX, int topY, int bottomX, int bottomY, bool isTopLeftZero = true);
    /**
     * Default constructor
     */
    Rectangle(){};
    /**
     * @param point
     * @return true if point is contained in the rectangle
     */
    bool contains(PointXY point);

    int getHeight() {return abs(topLeftCorner.y - bottomRightCorner.y);};
    int getWidth(){ return abs(topLeftCorner.x - bottomRightCorner.x); };
    PointXY getTopLeftCorner() { return topLeftCorner;};
    PointXY getBottomRightCorner() { return bottomRightCorner;};
    bool isTopLeftOrigin(){return isTopLeftZero;};

private:
    PointXY topLeftCorner;
    PointXY bottomRightCorner;
    bool isTopLeftZero;

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

    /**
     * Normalises the map so that its energy sums to 1.
     * @param outputMap optional parameter to output the normalised map. If not set this map is normalised
     */
    void normalise(vFeatureMap *outputMap = YARP_NULLPTR);

    /**
     * Finds the maximum in the map and outputs its coordinates
     * @param rowMax output row of max value
     * @param colMax output col of max value
     */
    void max(int &rowMax, int &colMax);

    /**
     * Computes the sum of the energy of every element in the map
     * @return total energy
     */
    double totalEnergy();
    /**
     * Computes the sum of the energy of the elements of a certain ROI in the map
     * @param ROI
     * @return energy in ROI
     */
    double energyInROI(Rectangle ROI);

    int getRowPadding(){ return rPadding;};
    int getColPadding(){ return cPadding;};

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
