#include "rigid2d.hpp"
#include <iostream>
#include <cmath>

namespace turtlelib{

    Transform2D::Transform2D(Vector2D v){
      T[0][0] = 1;
      T[0][1] = 0;  
      T[0][2] = v.x;  
      T[1][0] = 0;
      T[1][1] = 1; 
      T[1][2] = v.y; 
      T[2][0] = 0;
      T[2][1] = 0;
      T[2][2] = 1;
    };

    Transform2D::Transform2D(double radians){
      T[0][0] = cos(radians);
      T[0][1] = -1 * sin(radians);  
      T[0][2] = 0;  
      T[1][0] = sin(radians);
      T[1][1] = cos(radians); 
      T[1][2] = 0; 
      T[2][0] = 0;
      T[2][1] = 0;
      T[2][2] = 1;
    };

    Transform2D::Transform2D(Vector2D v, double radians){
      T[0][0] = cos(radians);
      T[0][1] = -1 * sin(radians);  
      T[0][2] = v.x;  
      T[1][0] = sin(radians);
      T[1][1] = cos(radians); 
      T[1][2] = v.y; 
      T[2][0] = 0;
      T[2][1] = 0;
      T[2][2] = 1;
    };

    Transform2D Transform2D::inv() const{

      Transform2D T_inv; 

      T_inv.T[0][0] = T[0][0];
      T_inv.T[0][1] = T[1][0];  
      T_inv.T[0][2] = -1 * T[0][2];  
      T_inv.T[1][0] = T[0][1];
      T_inv.T[1][1] = T[1][1]; 
      T_inv.T[1][2] = -1 * T[1][2]; 
      T_inv.T[2][0] = 0;
      T_inv.T[2][1] = 0;
      T_inv.T[2][2] = 1;

      return T_inv;
    };

};