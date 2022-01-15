#include "rigid2d.hpp"
#include <iostream>


int main() {
    
    double deg = 270;
    double rad = turtlelib::deg2rad(deg);
    deg = turtlelib::rad2deg(rad);
    double rad_pis = rad/3.14159265;
    std::cout << "Radians: " << rad_pis<<"pi" << "\n";
    std::cout << "Degrees: " << deg << "\n";
    return 0;

    turtlelib::Vector2D v;
    v.x = 1;
    v.y = 1;

    turtlelib::Transform2D Tab, Tac, Tba, Tbc, Tca, Tcb;
    
    turtlelib::Transform2D T(v, 3.14159265/2);
    turtlelib::Vector2D v1;
    v1.x = 2;
    v1.y = 2;
    turtlelib::Vector2D v1_new = T(v1);
    std::cout << v1_new;

    // std::cout << "Enter transform T_{a,b}" << std::endl;
    // std::cin >> Tab;

    // std::cout << "Enter transform T_{b,c}" << std::endl;
    // std::cin >> Tbc;

    return 0;
};

