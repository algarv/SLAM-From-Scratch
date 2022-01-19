#include "rigid2d.hpp"
#include <iostream>

double deg;
double x_in;
double y_in; 

int main() {
    
    // double ex_deg = 270;
    // double ex_rad = turtlelib::deg2rad(ex_deg);
    // ex_deg = turtlelib::rad2deg(ex_rad);
    // double rad_pis = ex_rad/3.14159265;
    // std::cout << "Radians: " << rad_pis<<"pi" << "\n";
    // std::cout << "Degrees: " << ex_deg << "\n";

    turtlelib::Transform2D Tab(0), Tbc(0), Tac(0), Tba(0), Tcb(0), Tca(0);

    std::cout << "Enter transform T_{a,b}:" << std::endl;
    std::cin >> Tab;
    std::cout << "Enter transform T_{b,c}:" << std::endl;
    std::cin >> Tbc;

    Tac = Tab; 
    Tba = Tbc;

    // Tbc * Tca
    Tba*=(Tac.inv());    

    // Tab * Tbc;
    Tac*=Tbc;

    Tcb = Tbc.inv();

    Tca = Tac.inv();

    std::cout << "T_{a,b}: " << Tab; 
    std::cout << "T_{b,a}: " << Tba; 
    std::cout << "T_{b,c}: " << Tbc; 
    std::cout << "T_{c,b}: " << Tcb; 
    std::cout << "T_{a,c}: " << Tac; 
    std::cout << "T_{c,a}: " << Tca; 

    std::cout << "Enter vector v_b:\n" << std::endl;
    
    return 0;
};

