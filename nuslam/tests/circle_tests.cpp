// #include <ros/ros.h>
#include <catch_ros/catch.hpp>
#include <math.h>
#include <vector>
#include <ros/console.h>
#include "measurement/measurement.hpp"
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <sstream>

TEST_CASE("Landmark Tests", "[Tests]") { // Anna Garverick
        
    SECTION("Test 1") {
        
        measurement::clustering cluster;

        std::vector<measurement::cluster> found_clusters;
        measurement::cluster new_cluster;
       
        measurement::point cluster_point = {.x = 1, .y = 7};
        new_cluster.pt.push_back(cluster_point);
        cluster_point = {.x = 2, .y = 6};
        new_cluster.pt.push_back(cluster_point);
        cluster_point = {.x = 5, .y = 8};
        new_cluster.pt.push_back(cluster_point);
        cluster_point = {.x = 7, .y = 7};
        new_cluster.pt.push_back(cluster_point);
        cluster_point = {.x = 9, .y = 5};
        new_cluster.pt.push_back(cluster_point);
        cluster_point = {.x = 3, .y = 7};
        new_cluster.pt.push_back(cluster_point);

        found_clusters.push_back(new_cluster);
        for (int j = 0; j < found_clusters.size(); j++){
            for (int i = 0; i < found_clusters[j].pt.size(); i++){
                ROS_WARN("cluster(%d,%d) input x: %3.6f",j,i,found_clusters[j].pt[i].x);
                ROS_WARN("cluster input y: %3.6f",found_clusters[j].pt[i].y);
            }
        }

        std::vector<measurement::circle> potential_circles = cluster.circle_detection(found_clusters);
        
        std::vector<measurement::circle> confirmed_circles = cluster.circle_classification(found_clusters, potential_circles);


        REQUIRE(confirmed_circles[0].a == Approx(4.615482).margin(.001));
        REQUIRE(confirmed_circles[0].b == Approx(2.80754).margin(.001));
        REQUIRE(confirmed_circles[0].R2 == Approx(4.8275).margin(.001));
    }

    SECTION("Test 2") {

        measurement::clustering cluster;

        std::vector<measurement::cluster> found_clusters;
        measurement::cluster new_cluster;
        new_cluster.pt = {{.x = -1, .y = 0}, {.x = -0.3, .y = -0.06}, {.x = 0.3, .y = 0.1}, {.x = 1, .y = 0}};
        found_clusters.push_back(new_cluster);

        std::vector<measurement::circle> potential_circles = cluster.circle_detection(found_clusters);
        
        std::vector<measurement::circle> confirmed_circles = cluster.circle_classification(found_clusters, potential_circles);


        REQUIRE(confirmed_circles[0].a == Approx(4.615482).margin(0.4908357));
        REQUIRE(confirmed_circles[0].b == Approx(2.80754).margin(-22.15212));
        REQUIRE(confirmed_circles[0].R2 == Approx(4.8275).margin(22.17979));

    }

}
