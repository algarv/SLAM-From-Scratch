#ifndef MEASUREMENT_INCLUDE_GUARD_HPP
#define MEASUREMENT_INCLUDE_GUARD_HPP 

#include <vector>
#include <armadillo>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"

#define threshold .05
#define obj_radius .125

struct point{
    double x;
    double y;
};

struct cluster{
    std::vector<point> pt;
};

struct circle{
    double a;
    double b;
    double R2;
};

std::vector<cluster> clusters;
std::vector<point> centroids;
std::vector<double> z_bar;
std::vector<cluster> found_clusters;
point saved_pt;


std::vector<circle> circle_detection(std::vector<cluster> clusters);
std::vector<circle> circle_classification(std::vector<cluster> found_clusters, std::vector<circle> circles);

#endif