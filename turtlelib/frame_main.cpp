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
    double rad = turtlelib::deg2rad(deg);
    turtlelib::Transform2D Tab(Vab,rad);
    
    std::cout << "Enter transform T_{b,c}:\n" << std::endl;
    std::cout << "\r deg: ";
    std::cin >> deg;
    std::cout << "\r x: "; 
    std::cin >> x_in;
    std::cout << "\r y: ";
    std::cin >> y_in;

    turtlelib::Vector2D Vbc;
    Vbc.x = x_in;
    Vbc.y = y_in;
    rad = turtlelib::deg2rad(deg);
    turtlelib::Transform2D Tbc(Vbc,rad);

    // turtlelib::Transform2D Tac = Tab*=Tbc;

    std::cout << "T_{A,B}: deg: " << turtlelib::rad2deg(Tab.rotation()) << " x: " << Tab.translation().x << " y: " << Tab.translation().y << "\n";
    std::cout << "T_{B,C}: deg: " << turtlelib::rad2deg(Tbc.rotation()) << " x: " << Tbc.translation().x << " y: " << Tbc.translation().y << "\n";
    // std::cout << "T_{B,C}: deg: " << turtlelib::rad2deg(Tac.rotation()) << " x: " << Tac.translation().x << " y: " << Tac.translation().y << "\n";

    turtlelib::Vector2D V_b;
    std::cout << "Enter vector v_b:\n" << std::endl;
    std::cout << "\rv_b.x: ";
    std::cin >> V_b.x;
    std::cout << "\rv_b.y: "; 
    std::cin >> V_b.y;

    std::cout << "v_bhat: " << Tab.normalize(V_b);

    return 0;
};

