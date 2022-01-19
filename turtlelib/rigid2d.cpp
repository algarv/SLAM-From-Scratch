#include "rigid2d.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

namespace turtlelib{

    // Transform2D::Transform2D(){
    //   T[0][0] = 1;
    //   T[0][1] = 0;  
    //   T[0][2] = 0;  
    //   T[1][0] = 0;
    //   T[1][1] = 1; 
    //   T[1][2] = 0; 
    //   T[2][0] = 0;
    //   T[2][1] = 0;
    //   T[2][2] = 1;
    // };

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
    //   // -T[1][2]   T[0][2]     0         0       0    1 

    //   adj[3][0] = 0;
    //   adj[3][1] = 0;
    //   adj[3][2] = T[1][2];
    //   adj[4][0] = 0;
    //   adj[4][1] = 0;
    //   adj[4][2] = -T[0][2];
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
      // T[0][0] T[0][1]  0         0        0     0     *   0
      // T[1][0] T[1][1]  0         0        0     0         0
      //    0       0     1         0        0     0        t.w
      //    0       0  T[1][2]   T[0][0]  T[0][1]  0        t.vx
      //    0       0 -T[0][2]   T[1][0]  T[1][1]  0        t.vy
      //    *       *     0         0        0     1         0

      Twist2D t_new;
      t_new.w = t.w;
      t_new.vx = T[1][2]*t.w + T[0][0]*t.vx + T[0][1]*t.vy;
      t_new.vy = -T[0][2]*t.w + T[1][0]*t.vx + T[1][1]*t.vy;

      return t_new;
    }

    Transform2D Transform2D::inv() const{

      // Vector2D trans;
      // trans.x = -1 * T[0][2];
      // trans.y = -1 * T[1][2];
      // double radians = -1*asin(T[0][0]);
      // Transform2D T_inv(trans, radians); 

      Transform2D T_inv(0);
    
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

    Transform2D & Transform2D::operator*=(const Transform2D & rhs){

      // T[0][0] T[0][1] T[0][2] * rhs.T[0][0] T[0][1] T[0][2]
      // T[1][0] T[1][1] T[1][2]       T[1][0] T[1][1] T[1][2]
      // T[2][0] T[2][1] T[2][2]       T[2][0] T[2][1] T[2][2]

      const float a = T[0][0];
      const float b = T[0][1];
      const float c = T[0][2];
      const float d = T[1][0];
      const float e = T[1][1];
      const float f = T[1][2];
      const float g = T[2][0];
      const float h = T[2][1];
      const float i = T[2][2];

      T[0][0] = a*rhs.T[0][0] + b*rhs.T[1][0] + c*rhs.T[2][0];
      T[0][1] = a*rhs.T[0][1] + b*rhs.T[1][1] + c*rhs.T[2][1];
      T[0][2] = a*rhs.T[0][2] + b*rhs.T[1][2] + c*rhs.T[2][2];
      T[1][0] = d*rhs.T[0][0] + e*rhs.T[1][0] + f*rhs.T[2][0];
      T[1][1] = d*rhs.T[0][1] + e*rhs.T[1][1] + f*rhs.T[2][1];
      T[1][2] = d*rhs.T[0][2] + e*rhs.T[1][2] + f*rhs.T[2][2];
      T[2][0] = g*rhs.T[0][0] + h*rhs.T[1][0] + i*rhs.T[2][0];
      T[2][1] = g*rhs.T[0][1] + h*rhs.T[1][1] + i*rhs.T[2][1];
      T[2][2] = g*rhs.T[0][2] + h*rhs.T[1][2] + i*rhs.T[2][2];
    
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

    std::istream & operator>>(std::istream & is, Transform2D & tf){
      double degrees, radians;
      char str[2];
      char delim = ' ';
      int num;
      std::streamsize n = 4;
      Vector2D v;
      
      // std::cin >> std::ws; //remove whitespace
      
      std::cin.ignore(10,' ');
      std::cin.get(str, n, delim);
      degrees = ((int) str[0] - 48)*10 + ((int) str[1] - 48)*1;
      
      std::cin.ignore(10,' ');
      std::cin.get(str, n, delim);
      v.x = (int) str[1] - 48;
      
      std::cin.ignore(10,' ');
      std::cin.get(str, n, delim);
      v.y = (int) str[0] - 48;

      radians = deg2rad(degrees);

      turtlelib::Transform2D T(v,radians);
      tf = T;
      // is >> tf;
      return is;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
      os << "deg: " << rad2deg(tf.rotation()) << " x: " << tf.translation().x << " y: " << tf.translation().y << "\n";
      return os;
    }

};