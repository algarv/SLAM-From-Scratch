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

    // double Transform2D::adjoint() const{
      
    //   double adj[4][4];

    //   adj[0][0] = T[0][0];
    //   adj[0][1] = T[0][1];
    //   adj[0][2] = 0;
    //   adj[1][0] = T[1][0];
    //   adj[1][1] = T[1][1];
    //   adj[1][2] = 0;
    //   adj[2][0] = 0;
    //   adj[2][1] = 0;
    //   adj[2][2] = 0;

    //   adj[3][3] = T[0][0];
    //   adj[3][4] = T[0][1];
    //   adj[3][5] = 0;
    //   adj[4][3] = T[1][0];
    //   adj[4][4] = T[1][1];
    //   adj[4][5] = 0;
    //   adj[5][3] = 0;
    //   adj[5][4] = 0;
    //   adj[5][5] = 0;

    //   //    0         0      T[1][2] * T[0][0] T[0][1] 0
    //   //    0         0     -T[0][2]   T[1][0] T[1][1] 0
    //   // -T[1][2]   T[0][2]     0         0       0    0 

    //   adj[3][0] = 0;
    //   adj[3][1] = 0;
    //   adj[3][2] = 0;
    //   adj[4][0] = 0;
    //   adj[4][1] = 0;
    //   adj[4][2] = 0;
    //   adj[5][0] = -1*T[1][2]*T[0][0] + T[0][2]*T[1][0];
    //   adj[5][1] = -1*T[1][2]*T[0][1] + T[0][2]*T[1][1];
    //   adj[5][2] = 0;

    //   adj[0][3] = 0;
    //   adj[0][4] = 0;
    //   adj[0][5] = 0;
    //   adj[1][3] = 0;
    //   adj[1][4] = 0;
    //   adj[1][5] = 0;
    //   adj[2][3] = 0;
    //   adj[2][4] = 0;
    //   adj[2][5] = 0;

    //   return adj;
    // }

    Vector2D Transform2D::operator()(Vector2D v) const{
      
      // T[0][0] T[0][1] T[0][2] * V[0]
      // T[1][0] T[1][1] T[1][2]   V[1]
      // T[2][0] T[2][1] T[2][2]    1

      Vector2D v_trans;   
      v_trans.x = T[0][0] * v.x + T[0][1] * v.y + T[0][2];
      v_trans.y = T[1][0] * v.x + T[1][1] * v.y + T[1][2];
      
      return v_trans;

    };

    Twist2D Transform2D::operator()(Twist2D t) const{
      // T[0][0] T[0][1]  0      0        0     0     *   0
      // T[1][0] T[1][1]  0      0        0     0         0
      //    0       0     1      0        0     0        t.w
      //    0       0     0   T[0][0]  T[0][1]  0        t.vx
      //    0       0     0   T[1][0]  T[1][1]  0        t.vy
      //    *       *     0      0        0     1         0

      Twist2D t_new;
      t_new.w = t.w;
      t_new.vx = T[0][0]*t.vx + T[0][1]*t.vy;
      t_new.vy = T[1][0]*t.vx + T[1][1]*t.vy;

      return t_new;
    }

    Transform2D Transform2D::inv() const{

      Vector2D trans;
      trans.x = -1 * T[0][2];
      trans.y = -1 * T[1][2];
      double radians = -1*asin(T[0][0]);
      Transform2D T_inv(trans, radians); 

      // Transform2D T_inv;
    
      // T_inv.T[0][0] = T[0][0];
      // T_inv.T[0][1] = T[1][0];  
      // T_inv.T[0][2] = -1 * T[0][2];  
      // T_inv.T[1][0] = T[0][1];
      // T_inv.T[1][1] = T[1][1]; 
      // T_inv.T[1][2] = -1 * T[1][2]; 
      // T_inv.T[2][0] = 0;
      // T_inv.T[2][1] = 0;
      // T_inv.T[2][2] = 1;

      return T_inv;
    };

    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
      // Transform2D combined(0);
      // Transform2D *combinedp;

      // T[0][0] T[0][1] T[0][2] * rhs.T[0][0] T[0][1] T[0][2]
      // T[1][0] T[1][1] T[1][2]       T[1][0] T[1][1] T[1][2]
      // T[2][0] T[2][1] T[2][2]       T[2][0] T[2][1] T[2][2]

      T[0][0] = T[0][0]*rhs.T[0][0] + T[0][1]*rhs.T[1][0] + T[0][2]*rhs.T[2][0];
      T[0][1] = T[0][0]*rhs.T[0][1] + T[0][1]*rhs.T[1][1] + T[0][2]*rhs.T[2][1];
      T[0][2] = T[0][0]*rhs.T[0][2] + T[0][1]*rhs.T[1][2] + T[0][2]*rhs.T[2][2];
      T[1][0] = T[1][0]*rhs.T[0][0] + T[1][1]*rhs.T[1][0] + T[1][2]*rhs.T[2][0];
      T[1][1] = T[1][0]*rhs.T[0][1] + T[1][1]*rhs.T[1][1] + T[1][2]*rhs.T[2][1];
      T[1][2] = T[1][0]*rhs.T[0][2] + T[1][1]*rhs.T[1][2] + T[1][2]*rhs.T[2][2];
      T[2][0] = T[2][0]*rhs.T[0][0] + T[2][1]*rhs.T[1][0] + T[2][2]*rhs.T[2][0];
      T[2][1] = T[2][0]*rhs.T[0][1] + T[2][1]*rhs.T[1][1] + T[2][2]*rhs.T[2][1];
      T[2][2] = T[2][0]*rhs.T[0][2] + T[2][1]*rhs.T[1][2] + T[2][2]*rhs.T[2][2];
    
      return *this;
    }

    Vector2D Transform2D::translation() const{
      Vector2D translation;
      translation.x = T[0][2];
      translation.y = T[1][2];

      return translation;
    }

    double Transform2D::rotation() const{
      double angle;
      angle = acos(T[0][0]);

      return angle;
    }

    Vector2D Transform2D::normalize(Vector2D v) const{
      Vector2D normalized;

      double m = sqrt(v.x*v.x + v.y*v.y);
      normalized.x = v.x/m;
      normalized.y = v.y/m;

      return normalized;
    }
}
;