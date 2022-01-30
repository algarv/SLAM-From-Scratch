#include "../include/turtlelib/rigid2d.hpp" /// rec: This should compile with #include<turtlelib/rigid2d.hpp>
#include <iostream>

/// rec: why are these global variables? If you must use a global variable, make it static
double deg; 
double x_in;
double y_in; 

int main() {
    /// rec: it would be better to declare some of these where they are used rather than all up-front
    turtlelib::Transform2D Tab(0), Tbc(0), Tac(0), Tba(0), Tcb(0), Tca(0);

    std::cout << "Enter transform T_{a,b}:" << std::endl;
    std::cin >> Tab;
    std::cout << "Enter transform T_{b,c}:" << std::endl;
    std::cin >> Tbc;

    Tba = Tab.inv();    
    Tac = Tab * Tbc;
    Tcb = Tbc.inv();
    Tca = Tac.inv();

    std::cout << "T_{a,b}: " << Tab; 
    std::cout << "T_{b,a}: " << Tba; 
    std::cout << "T_{b,c}: " << Tbc; 
    std::cout << "T_{c,b}: " << Tcb; 
    std::cout << "T_{a,c}: " << Tac; 
    std::cout << "T_{c,a}: " << Tca; 

    turtlelib::Vector2D va, vb, vc, vb_hat;

    std::cout << "Enter vector v_b:" << std::endl;
    std::cin >> vb;
    
    vb_hat = normalize(vb);
    va = Tab(vb);
    vc = Tcb(vb);
    
    std::cout << "vb_hat: " << vb_hat;
    std::cout << "v_a: " << va;
    std::cout << "v_b: " << vb;
    std::cout << "v_c: " << vc;

    turtlelib::Twist2D Va, Vb, Vc;

    std::cout << "Enter twist V_b:" << std::endl;
    std::cin >> Vb;

    Va = Tab(Vb);
    Vc = Tca(Va);

    std::cout << "Va: " << Va;
    std::cout << "Vb: " << Vb;
    std::cout << "Vc: " << Vc;

    return 0;
};

