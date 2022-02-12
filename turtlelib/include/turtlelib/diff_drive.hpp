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

    /// \brief Calculates the kinematics of a differential drive robot
    class DiffDrive{
        public: 
            /// \brief Sets the configuration to (0, 0, 0)
            DiffDrive();
            
            /// \brief Calculates the angles of the left and right wheel joints
            /// \param old_phi - the angle of the wheel joints prior to the update
            /// \param twist - the twist of the robot
            Wheel_Angle wheel_pos(Wheel_Angle old_phi, Twist2D twist);

            /// \brief Calculates the angle velocities of the left and right wheels
            /// \param twist - the twist of the robot
            Wheel_Angular_Velocities wheel_vel(Twist2D twist);

            /// \brief Calculates the forward kinematics of the robot
            /// \param new_wheel_angles - the updated wheel angles
            /// \param old_wheel_angles - the previous wheel angles
            Twist2D get_twist(Wheel_Angle new_wheel_angles, Wheel_Angle old_wheel_angles);

            /// \brief Calculates the forward kinematics of the robot
            /// \param wheel_vel - the wheel velocities
            Twist2D get_twist(Wheel_Angular_Velocities wheel_vel);

            /// \brief Calculates the inverse kinematics of the robot
            /// \param twist - the twist of the robot
            /// \param old_pos - the previous pose of the robot
            q get_q(Twist2D twist, q old_pos);

            /// \brief Calculates the inverse kinematics of the robot
            /// \param new_wheel_angles - the updated wheel angles
            /// \param old_wheel_angles - the previous wheel angles
            q get_q(Wheel_Angle new_wheel_angles, Wheel_Angle old_wheel_angles, q old_pos); 
        private:
            q config;       

    };



}
#endif