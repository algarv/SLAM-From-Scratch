#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

constexpr double WHEEL_RADIUS = .033;
constexpr double TRACK_WIDTH = .16;


namespace turtlelib
{

    DiffDrive::DiffDrive(){
        config.theta = 0;
        config.x = 0;
        config.y = 0;
    }

    Wheel_Angular_Velocities DiffDrive::wheel_vel(Twist2D twist){
        
        Wheel_Angular_Velocities dw;

        dw.L = (twist.vx - twist.w * TRACK_WIDTH) / WHEEL_RADIUS;
        dw.R = (twist.w * TRACK_WIDTH + twist.vx) / WHEEL_RADIUS;
        
        return dw;
    }

    Wheel_Angle DiffDrive::wheel_pos(Wheel_Angle old_phi, Twist2D twist){
        
        Wheel_Angle wheel_pos;
        
        Wheel_Angular_Velocities dphi = wheel_vel(twist);

        wheel_pos.L = old_phi.L + dphi.L;
        wheel_pos.R = old_phi.L + dphi.R;
        
        return wheel_pos;
    }

    q DiffDrive::get_q(Twist2D twist, q old_pos){

        q q;

        Vector2D old_pos_vec;
        Vector2D new_pos_vec;
        old_pos_vec.x = old_pos.x;
        old_pos_vec.y = old_pos.y;

        Transform2D Tbbp = integrate_twist(twist);

        new_pos_vec = Tbbp(old_pos_vec);

        q.theta = old_pos.theta + twist.w;
        q.x = new_pos_vec.x;
        q.y = new_pos_vec.y;

        return q;
    }

    q DiffDrive::get_q(Wheel_Angle new_wheel_angles, Wheel_Angle old_wheel_angles, q old_pos){

        q q;

        Wheel_Angle dphi;
        Twist2D twist;

        dphi.L = new_wheel_angles.L - old_wheel_angles.L;
        dphi.R = new_wheel_angles.R - old_wheel_angles.R;

        twist.w = (WHEEL_RADIUS / TRACK_WIDTH) * (dphi.L - dphi.R);
        twist.vx = .5*WHEEL_RADIUS*(dphi.R + dphi.L);
        twist.vy = 0;

        q = get_q(twist, old_pos);

        return q;
    }


}