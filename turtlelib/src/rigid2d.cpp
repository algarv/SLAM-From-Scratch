#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <fstream>
#include <string>
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
    
      // r.T = T[0][0] T[1][0]
      //       T[0][1] T[1][1]
      // t = -r.T * T[0][2]
      //            T[1][2] 

      T_inv.T[0][0] = T[0][0];
      T_inv.T[0][1] = T[1][0];  
      T_inv.T[0][2] = -1*(T[0][0]*T[0][2] + T[1][0]*T[1][2]);  
      T_inv.T[1][0] = T[0][1];
      T_inv.T[1][1] = T[1][1]; 
      T_inv.T[1][2] = -1*(T[0][1]*T[0][2] + T[1][1]*T[1][2]);
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

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        lhs*=rhs;
        return lhs;
    }

    Vector2D Transform2D::translation() const{
      Vector2D translation;
      translation.x = T[0][2];
      translation.y = T[1][2];

      return translation;
    }

    double Transform2D::rotation() const{
      double angle;
      
      angle = atan2(T[1][0],T[0][0]);

      return angle;
    }

    Vector2D normalize(Vector2D v){
      Vector2D normalized;

      double m = sqrt(v.x*v.x + v.y*v.y);

      normalized.x = v.x/m;
      normalized.y = v.y/m;

      return normalized;
    }

    double normalize_angle(double rad){
      double norm_angle;
      while (rad >= 2*turtlelib::PI){
        rad -= 2*turtlelib::PI;
      }

      if (rad > PI) {
        norm_angle = -1*PI + (rad - PI);
      }
      if (rad <= -1*PI) {
        norm_angle = PI + (rad + PI);
      }
      return norm_angle;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf){
      double degrees, radians;
      Vector2D v;

      is.ignore(10,'d');       
      is.ignore(10,' ');
      is >> degrees;
      
      is.ignore(10,' ');
      is.ignore(10,' ');
      is >> v.x;
      
      is.ignore(10,':');
      is >> v.y;

      is.get();

      radians = deg2rad(degrees);

      turtlelib::Transform2D T(v,radians);
      tf = T;      
      return is;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
      os << "deg: " << rad2deg(tf.rotation()) << " x: " << tf.translation().x << " y: " << tf.translation().y << "\n";
      return os;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v){
      double x, y;
      
      char c = is.peek();
      if (c == '[') {
        is.get();
      }
      
      // std::cin.ignore(10,' ');        
      
      is >> x;
      is >> y;
      
      // std::cout << x << " " << y;

      v.x = x;
      v.y = y;
      is.get();

      return is;
    }

    std::ostream & operator<<(std::ostream &os, const Vector2D &v){
      os << "[" << v.x << " " << v.y << "]" << "\n";
      return os;
    }

    std::istream & operator>>(std::istream & is, Twist2D & v){
      
      char c = is.peek();
      if (c == '[') {
        is.get();
      }

      is >> v.w;
      is >> v.vx;
      is >> v.vy;

      is.get();

      return is;
    }

    std::ostream & operator<<(std::ostream &os, const Twist2D &v){
      os << "[" << v.w << " " << v.vx << " " << v.vy << "]" << "\n";
      return os;
    }

    Vector2D operator+=(Vector2D lhs, const Vector2D rhs){
      
      lhs.x = lhs.x + rhs.x;
      lhs.y = lhs.y + rhs.y;

      return lhs;
    }

    Vector2D operator+(Vector2D lhs, const Vector2D rhs){
      
      lhs.x = lhs.x + rhs.x;
      lhs.y = lhs.y + rhs.y;

      return lhs;
    }

    Vector2D operator-=(Vector2D lhs, const Vector2D rhs){
      
      lhs.x = lhs.x - rhs.x;
      lhs.y = lhs.y - rhs.y;

      return lhs;
    }

    Vector2D operator-(Vector2D lhs, const Vector2D rhs){
      
      lhs.x = lhs.x - rhs.x;
      lhs.y = lhs.y - rhs.y;

      return lhs;
    }

    Vector2D operator*=(Vector2D lhs, const int num){
      
      lhs.x = lhs.x * num;
      lhs.y = lhs.y * num;

      return lhs;
    }

    Vector2D operator*(Vector2D lhs, const int num){
      
      lhs.x = lhs.x * num;
      lhs.y = lhs.y * num;

      return lhs;
    }

    Vector2D operator*=(const int num, Vector2D lhs){
      
      lhs.x = lhs.x * num;
      lhs.y = lhs.y * num;

      return lhs;
    }

    Vector2D operator*(const int num, Vector2D lhs){
      
      lhs.x = lhs.x * num;
      lhs.y = lhs.y * num;

      return lhs;
    }

    double dot(Vector2D vec1, Vector2D vec2){
      double dot_product;
      dot_product = vec1.x*vec2.x + vec1.y*vec2.y;
      return dot_product;
    } 

    double magnitude(Vector2D vec){
      double mag;
      mag = sqrt(vec.x*vec.x + vec.y*vec.y);
      return mag;
    } 

    double angle(Vector2D vec1, Vector2D vec2){
      double dot_product = dot(vec1,vec2);
      double mag1 = magnitude(vec1);
      double mag2 = magnitude(vec2);
      double radians = acos(dot_product / (mag1*mag2));
      return radians;
    }

    Transform2D integrate_twist(Twist2D twist){

      double dx = twist.vx;
      double dy = twist.vy;
      double dth = twist.w;


      Vector2D Vsb;

      if (dth == 0){
        Vsb.x = dx;
        Vsb.y = dy;

        Transform2D Tbbp(Vsb);

        return Tbbp;
      }
      else {
        Vsb.x = dy / dth;
        Vsb.y = -1*dx / dth;

        Transform2D Tsb(Vsb);

        Transform2D Tssp(dth);

        Transform2D Tbs = Tsb.inv();

        Transform2D Tbbp = (Tbs*Tssp)*Tsb;

        return Tbbp;
      }

    }

};