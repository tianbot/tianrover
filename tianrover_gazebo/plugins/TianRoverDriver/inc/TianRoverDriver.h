/*
 * Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/

#ifndef GAZEBO_ROS_SKID_STEER_DRIVE_H_
#define GAZEBO_ROS_SKID_STEER_DRIVE_H_

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#include <termio.h>
#include <stdio.h>
#include <unistd.h>

namespace gazebo
{
    class Joint;
    class Entity;

    class GazeboRosSkidSteerDrive : public ModelPlugin
    {
        public:
            GazeboRosSkidSteerDrive();
            ~GazeboRosSkidSteerDrive();
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        protected:
            virtual void UpdateChild();

        private:
            void publishOdometry(double step_time);
            void getWheelVelocities();

            void setCommandVelocity(double linear_velocity, double yaw_rate);
            void setAckermannCommand(double speed, double steering_angle);
            void setAckermannSteeringFromRadius(double turn_radius);
            void setAckermannSteeringFromInnerAngle(double inner_angle, bool turn_left);
            double wheelLinearSpeed(double x, double y) const;
            void applyRockerDifferential();

            physics::WorldPtr world;
            physics::ModelPtr parent;
            physics::JointControllerPtr joint_controller_;
            event::ConnectionPtr update_connection_;

            std::string RFWheelJointName;
            std::string LFWheelJointName;
            std::string RMWheelJointName;
            std::string LMWheelJointName;
            std::string RBWheelJointName;
            std::string LBWheelJointName;
            std::string RFBogieJointName;
            std::string LFBogieJointName;
            std::string RBBogieJointName;
            std::string LBBogieJointName;
            std::string right_rocker_joint_name_;
            std::string left_rocker_joint_name_;

            double wheel_separation_;
            double front_wheel_separation_;
            double middle_wheel_separation_;
            double rear_wheel_separation_;
            double wheel_diameter_;
            double torque_;
            double wheel_base_;
            double steering_p_gain_;
            double steering_i_gain_;
            double steering_d_gain_;
            double steering_torque_;
            double steering_rate_;
            double applied_alpha_;
            double applied_beta_;
            double applied_rear_alpha_;
            double applied_rear_beta_;
            double rocker_differential_p_gain_;
            double rocker_differential_d_gain_;
            double rocker_differential_torque_;
            double rocker_differential_target_;

            physics::JointPtr joints[10];
            physics::JointPtr right_rocker_joint_;
            physics::JointPtr left_rocker_joint_;

            gazebo_ros::Node::SharedPtr rosnode_;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
            rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_subscriber_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rover_motion_mode_subscriber_;
            std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
            double command_timeout_ = 0.5;
            double last_command_time_ = -1.0;
            nav_msgs::msg::Odometry odom_;
            nav_msgs::msg::Path path_;
            std::string tf_prefix_;
            bool broadcast_tf_;

            boost::mutex lock;

            std::string robot_namespace_;
            std::string command_topic_;
            std::string ackermann_topic_;
            std::string rover_motion_mode_topic_;
            std::string rover_motion_mode_;
            std::string odometry_topic_;
            std::string path_topic_;
            std::string odometry_frame_;
            std::string robot_base_frame_;

            void cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr& cmd_msg);
            void ackermannCallback(const ackermann_msgs::msg::AckermannDrive::ConstSharedPtr& ackermann_msg);
            void roverMotionModeCallback(const std_msgs::msg::String::ConstSharedPtr& mode_msg);

            double x_,rot_;
            double wheel_speed_[6];
            double alpha_,beta_;
            double rear_alpha_, rear_beta_;
            double turn_radius_;
            bool isCMD;

            double update_rate_;
            double update_period_;
            common::Time last_update_time_;

            double covariance_x_;
            double covariance_y_;
            double covariance_yaw_;

            Eigen::Isometry3d T_bp_ = Eigen::Isometry3d::Identity();

    };
}
#endif
