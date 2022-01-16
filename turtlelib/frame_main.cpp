#include "rigid2d.hpp"
#include <iostream>

double deg;
double x_in;
double y_in; 

int main() {
    
    // double deg = 270;
    // double rad = turtlelib::deg2rad(deg);
    // deg = turtlelib::rad2deg(rad);
    // double rad_pis = rad/3.14159265;
    // std::cout << "Radians: " << rad_pis<<"pi" << "\n";
    // std::cout << "Degrees: " << deg << "\n";

    std::cout << "Enter transform T_{a,b}:\n" << std::endl;
    std::cout << "\r deg: ";
    std::cin >> deg;
    std::cout << "\r x: "; 
    std::cin >> x_in;
    std::cout << "\r y: ";
    std::cin >> y_in;

    turtlelib::Vector2D Vab;
    Vab.x = x_in;
    Vab.y = y_in;
    turtlelib::Transform2D Tab(Vab,deg);


    // turtlelib::Transform2D transform2d(v, turtlelib::PI/2);
    // turtlelib::Vector2D v1;
    // v1.x = 2;
    // v1.y = 2;
    // turtlelib::Vector2D v1_new = transform2d(v1);
    // std::cout << v1_new;

    // std::cout << "Enter transform T_{a,b}" << std::endl;
    // std::cin >> Tab;

    // std::cout << "Enter transform T_{b,c}" << std::endl;
    // std::cin >> Tbc;

    return 0;
};

