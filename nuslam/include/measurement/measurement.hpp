#ifndef MEASUREMENT_INCLUDE_GUARD_HPP
#define MEASUREMENT_INCLUDE_GUARD_HPP 

#include <vector>
#include <armadillo>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"

#define threshold .05
#define obj_radius .125

namespace measurement
{

struct point{
    double x;
    double y;
};

struct cluster{
    std::vector<measurement::point> pt;
};

struct circle{
    double a;
    double b;
    double R2;
};

/// \brief The framework for processing data from a LiDAR sensor into cylindrical obstacle estimations

class clustering
{
public:

    std::vector<circle> circle_detection(std::vector<cluster> found_clusters);
    std::vector<circle> circle_classification(std::vector<cluster> found_clusters, std::vector<circle> circles);

private:

    std::vector<circle> circles;
    std::vector<circle> confirmed_circles;
};

}
#endif