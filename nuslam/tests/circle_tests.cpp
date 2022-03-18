// #include <ros/ros.h>
#include <catch_ros/catch.hpp>
#include <math.h>
#include <vector>
#include <ros/console.h>
#include "measurement/measurement.hpp"
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <sstream>

// COMMENT OUT RADIUS AND STD FILTERS TO PASS TESTS

TEST_CASE("Landmark Tests", "[Tests]") { // Anna Garverick
        
    SECTION("Test 1") {
        
        std::vector<cluster> found_clusters;
        cluster new_cluster;
       
        new_cluster.pt = {{.x = 1, .y = 7}, {.x = 2, .y = 6}, {.x = 5, .y = 8}, {.x = 7, .y = 7}, {.x = 9, .y = 5}, {.x = 3, .y = 7}};
        found_clusters = {new_cluster};

        std::vector<circle> potential_circles = circle_detection(found_clusters);
        std::cout << "Finished detection\n";
        std::vector<circle> confirmed_circles = circle_classification(found_clusters, potential_circles);
        std::cout << "Finished classification\n";

        std::cout << "Output: " << confirmed_circles[0].a << "\n";

        REQUIRE(confirmed_circles[0].a == Approx(4.615482).margin(.001));
        REQUIRE(confirmed_circles[0].b == Approx(2.80754).margin(.001));
        REQUIRE(sqrt(confirmed_circles[0].R2) == Approx(4.8275).margin(.001));
    }

    SECTION("Test 2") {

        std::vector<cluster> found_clusters;
        cluster new_cluster;
        
        new_cluster.pt = {{.x = -1, .y = 0}, {.x = -0.3, .y = -0.06}, {.x = 0.3, .y = 0.1}, {.x = 1, .y = 0}};
        found_clusters = {new_cluster};
        
        std::vector<circle> potential_circles = circle_detection(found_clusters);
        std::cout <<"Found " << potential_circles.size() << " potential circles \n";
        std::cout << "Finished detection\n";
        std::vector<circle> confirmed_circles = circle_classification(found_clusters, potential_circles);
        std::cout << "Finished classification\n";

        std::cout << "Output: " << confirmed_circles[0].a << "\n";

        REQUIRE(confirmed_circles[0].a == Approx(0.4908357).margin(.001));
        REQUIRE(confirmed_circles[0].b == Approx(-22.15212).margin(.001));
        REQUIRE(sqrt(confirmed_circles[0].R2) == Approx(22.17979).margin(.001));

    }

}
