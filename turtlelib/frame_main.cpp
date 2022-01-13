#include "rigid2d.hpp"
#include <iostream>


int main() {
    
    double deg = 45;
    double rad = turtlelib::deg2rad(deg);
    deg = turtlelib::rad2deg(rad);
    std::cout << "Radians: " << rad << "\n";
    std::cout << "Degrees: " << deg << "\n";
    return 0;

    turtlelib::Transform2D Tab, Tac, Tba, Tbc, Tca, Tcb;


    std::cout << "Enter transform T_{a,b}" << std::endl;
    std::cin >> Tab;

    std::cout << "Enter transform T_{b,c}" << std::endl;
    std::cin >> Tbc;

    return 0;
};

