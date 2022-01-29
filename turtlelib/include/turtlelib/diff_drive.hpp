#ifndef DIFFDRIVE_INCLUDE_GUARD_HPP
#define DIFFDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.

#include "turtlelib/rigid2d.hpp"

#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath>
#include<string> 

namespace turtlelib
{
    /// \brief A structure with the wheel angles
    struct Wheel_Angle
    {
        /// \brief the angle of the left wheel
        double L = 0.0;

        /// \brief the angle of the right wheel
        double R = 0.0;
    };

    /// \brief A structure with the wheel angular velocities
    struct Wheel_Angular_Velocities
    {
        /// \brief the angle of the left wheel angular velocities
        double L = 0.0;

        /// \brief the angle of the right wheel angular velocities
        double R = 0.0;
    };

    struct q
    {
        /// \brief the angle of the robot in the world frame
        double theta = 0.0;

        /// \brief the x position of the robot in the world frame
        double x = 0.0;

        /// \brief the y position of the robot in the world frame
        double y = 0.0;
    };

    class DiffDrive{

        Wheel_Angle wheel_pos(Wheel_Angle old_phi, Twist2D twist);

        Wheel_Angular_Velocities wheel_vel(Twist2D twist);

        q get_q(Twist2D twist);

        q get_q(Wheel_Angle new_wheel_pos, Wheel_Angle old_wheel_pos);        

    };



}
#endif