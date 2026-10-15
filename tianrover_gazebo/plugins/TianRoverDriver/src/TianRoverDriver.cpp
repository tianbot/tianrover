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

#include "TianRoverDriver.h"

#include <algorithm>
#include <cmath>

namespace gazebo
{

    enum
    {
        WRF = 0,
        WLF = 1,
        WRM = 2,
        WLM = 3,
        WRB = 4,
        WLB = 5,
        BRF = 6,
        BLF = 7,
        BRB = 8,
        BLB = 9,
    };

    GazeboRosSkidSteerDrive::GazeboRosSkidSteerDrive()
    {
        wheel_speed_[WRF] = 0;
        wheel_speed_[WLF] = 0;
        wheel_speed_[WRM] = 0;
        wheel_speed_[WLM] = 0;
        wheel_speed_[WRB] = 0;
        wheel_speed_[WLB] = 0;

        x_ = rot_ = alpha_ = beta_ = rear_alpha_ = rear_beta_ = turn_radius_ = 0;
        applied_alpha_ = applied_beta_ = applied_rear_alpha_ = applied_rear_beta_ = 0;
        rocker_differential_p_gain_ = 20.0;
        rocker_differential_d_gain_ = 2.0;
        rocker_differential_torque_ = 10.0;
        rocker_differential_target_ = 0.0;
        isCMD = false;
        rosnode_ = nullptr;
        transform_broadcaster_ = nullptr;
        rover_motion_mode_ = "ackermann";
    }

    GazeboRosSkidSteerDrive::~GazeboRosSkidSteerDrive()
    {
        update_connection_.reset();

    }

    void GazeboRosSkidSteerDrive::cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr& cmd_msg)
    {
        boost::mutex::scoped_lock scoped_lock(lock);
        last_command_time_ = rosnode_->now().seconds();
        setCommandVelocity(cmd_msg->linear.x, cmd_msg->angular.z);
    }

    void GazeboRosSkidSteerDrive::ackermannCallback(const ackermann_msgs::msg::AckermannDrive::ConstSharedPtr& ackermann_msg)
    {
        boost::mutex::scoped_lock scoped_lock(lock);
        last_command_time_ = rosnode_->now().seconds();
        setAckermannCommand(ackermann_msg->speed, ackermann_msg->steering_angle);
    }

    void GazeboRosSkidSteerDrive::roverMotionModeCallback(const std_msgs::msg::String::ConstSharedPtr& mode_msg)
    {
        boost::mutex::scoped_lock scoped_lock(lock);
        if(mode_msg->data == "ackermann" || mode_msg->data == "rotate" || mode_msg->data == "omni")
        {
            rover_motion_mode_ = mode_msg->data;
            RCLCPP_INFO(rosnode_->get_logger(), "TianRoverDriver motion mode set to %s", rover_motion_mode_.c_str());
        }
        else
        {
            RCLCPP_WARN(rosnode_->get_logger(), "TianRoverDriver motion mode set failed, only ackermann / rotate / omni supported!");
        }
    }

    void GazeboRosSkidSteerDrive::setAckermannCommand(double speed, double steering_angle)
    {
        double max_steering_angle = 0.785398163;
        max_steering_angle = rosnode_->get_parameter("max_steering_angle").as_double();

        x_ = speed;
        isCMD = false;

        steering_angle = std::max(-max_steering_angle, std::min(max_steering_angle, steering_angle));
        const double kEpsilon = 1.0e-6;
        if(std::abs(speed) < kEpsilon)
        {
            alpha_ = beta_ = rear_alpha_ = rear_beta_ = turn_radius_ = 0.0;
            wheel_speed_[WRF] = wheel_speed_[WRM] = wheel_speed_[WRB] = 0.0;
            wheel_speed_[WLF] = wheel_speed_[WLM] = wheel_speed_[WLB] = 0.0;
            return;
        }
        if(std::abs(steering_angle) < kEpsilon)
        {
            alpha_ = beta_ = rear_alpha_ = rear_beta_ = turn_radius_ = 0.0;
            return;
        }

        setAckermannSteeringFromInnerAngle(std::abs(steering_angle), steering_angle > 0.0);
    }

    void GazeboRosSkidSteerDrive::setCommandVelocity(double linear_velocity, double yaw_rate)
    {
        double max_steering_angle = 0.785398163;
        max_steering_angle = rosnode_->get_parameter("max_steering_angle").as_double();

        x_ = linear_velocity;
        isCMD = false;

        const double kEpsilon = 1.0e-6;
        if(std::abs(linear_velocity) < kEpsilon)
        {
            alpha_ = beta_ = rear_alpha_ = rear_beta_ = turn_radius_ = 0.0;
            if(std::abs(yaw_rate) < kEpsilon)
            {
                wheel_speed_[WRF] = wheel_speed_[WRM] = wheel_speed_[WRB] = 0.0;
                wheel_speed_[WLF] = wheel_speed_[WLM] = wheel_speed_[WLB] = 0.0;
            }
            else
            {
                double wheel_linear_velocity = yaw_rate * wheel_separation_ / 2.0;
                wheel_speed_[WRF] = wheel_speed_[WRM] = wheel_speed_[WRB] = wheel_linear_velocity;
                wheel_speed_[WLF] = wheel_speed_[WLM] = wheel_speed_[WLB] = wheel_linear_velocity;
                isCMD = true;
            }
            return;
        }

        if(std::abs(yaw_rate) < kEpsilon)
        {
            alpha_ = beta_ = rear_alpha_ = rear_beta_ = turn_radius_ = 0.0;
            return;
        }

        const double turn_radius = linear_velocity / yaw_rate;
        setAckermannSteeringFromRadius(turn_radius);
    }

    void GazeboRosSkidSteerDrive::setAckermannSteeringFromRadius(double turn_radius)
    {
        double max_steering_angle = 0.785398163;
        max_steering_angle = rosnode_->get_parameter("max_steering_angle").as_double();

        const double kEpsilon = 1.0e-6;
        const double half_wheel_base = wheel_base_ / 2.0;
        const double half_front_track = front_wheel_separation_ / 2.0;
        const double half_rear_track = rear_wheel_separation_ / 2.0;
        const double max_half_track = std::max(half_front_track, half_rear_track);
        const double min_turn_radius = max_half_track + half_wheel_base / std::tan(max_steering_angle);

        if(std::abs(turn_radius) < kEpsilon)
        {
            alpha_ = beta_ = rear_alpha_ = rear_beta_ = turn_radius_ = 0.0;
            return;
        }

        const double turn_direction = turn_radius > 0.0 ? 1.0 : -1.0;
        turn_radius_ = turn_direction * std::max(std::abs(turn_radius), min_turn_radius);

        const double front_x = half_wheel_base;
        const double rear_x = -half_wheel_base;
        const double right_front_y = -half_front_track;
        const double left_front_y = half_front_track;
        const double right_rear_y = -half_rear_track;
        const double left_rear_y = half_rear_track;

        alpha_ = std::atan(front_x / (turn_radius_ - right_front_y));
        beta_ = std::atan(front_x / (turn_radius_ - left_front_y));
        rear_alpha_ = std::atan(rear_x / (turn_radius_ - right_rear_y));
        rear_beta_ = std::atan(rear_x / (turn_radius_ - left_rear_y));
    }

    void GazeboRosSkidSteerDrive::setAckermannSteeringFromInnerAngle(double inner_angle, bool turn_left)
    {
        double max_steering_angle = 0.785398163;
        max_steering_angle = rosnode_->get_parameter("max_steering_angle").as_double();

        const double kEpsilon = 1.0e-6;
        inner_angle = std::max(-max_steering_angle, std::min(max_steering_angle, inner_angle));
        if(std::abs(inner_angle) < kEpsilon)
        {
            alpha_ = beta_ = rear_alpha_ = rear_beta_ = turn_radius_ = 0.0;
            return;
        }

        const double inner_radius = (wheel_base_ / 2.0) / std::tan(std::abs(inner_angle));
        const double center_radius = inner_radius + front_wheel_separation_ / 2.0;
        setAckermannSteeringFromRadius(turn_left ? center_radius : -center_radius);
    }

    void GazeboRosSkidSteerDrive::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {

        this->parent = _parent;
        this->world = _parent->GetWorld();
        rosnode_ = gazebo_ros::Node::Get(_sdf);
        rosnode_->declare_parameter("max_steering_angle", 0.785398163);
        command_timeout_ = _sdf->Get<double>("commandTimeout", 0.5).first;

        this->robot_namespace_ = rosnode_->get_namespace();
        this->broadcast_tf_ = false;
        if (!_sdf->HasElement("broadcastTF"))
        {
            if (!this->broadcast_tf_)
                RCLCPP_INFO(rosnode_->get_logger(), "RoverDriver Plugin (ns = %s) missing <broadcastTF>, defaults to false.",this->robot_namespace_.c_str());
            else
                RCLCPP_INFO(rosnode_->get_logger(), "RoverDriver Plugin (ns = %s) missing <broadcastTF>, defaults to true.",this->robot_namespace_.c_str());
        }
        else
            this->broadcast_tf_ = _sdf->GetElement("broadcastTF")->Get<bool>();
        this->update_rate_ = 100.0;
        if(!_sdf->HasElement("updateRate"))
            RCLCPP_WARN(rosnode_->get_logger(), "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <updateRate>, defaults to %f",
                this->robot_namespace_.c_str(), this->update_rate_);
        else
            this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();

        this->RFWheelJointName = "BogieRF_WheelRF";
        if(!_sdf->HasElement("RFWheelJoint"))
            RCLCPP_WARN(rosnode_->get_logger(), "TianRoverDriver Plugin (ns = %s) missing <RFWheelJoint>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->RFWheelJointName.c_str());
        else
            this->RFWheelJointName = _sdf->GetElement("RFWheelJoint")->Get<std::string>();
        this->LFWheelJointName = "BogieLF_WheelLF";
        if(!_sdf->HasElement("LFWheelJoint"))
            RCLCPP_WARN(rosnode_->get_logger(), "TianRoverDriver Plugin (ns = %s) missing <LFWheelJoint>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->LFWheelJointName.c_str());
        else
            this->LFWheelJointName = _sdf->GetElement("LFWheelJoint")->Get<std::string>();
        this->RMWheelJointName = "BogieRM_WheelRM";
        if(!_sdf->HasElement("RMWheelJoint"))
            RCLCPP_WARN(rosnode_->get_logger(), "TianRoverDriver Plugin (ns = %s) missing <RMWheelJoint>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->RMWheelJointName.c_str());
        else
            this->RMWheelJointName = _sdf->GetElement("RMWheelJoint")->Get<std::string>();
        this->LMWheelJointName = "BogieLM_WheelLM";
        if(!_sdf->HasElement("LMWheelJoint"))
            RCLCPP_WARN(rosnode_->get_logger(), "TianRoverDriver Plugin (ns = %s) missing <LMWheelJoint>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->LMWheelJointName.c_str());
        else
            this->LMWheelJointName = _sdf->GetElement("LMWheelJoint")->Get<std::string>();
        this->RBWheelJointName = "BogieRB_WheelRB";
        if(!_sdf->HasElement("RBWheelJoint"))
            RCLCPP_WARN(rosnode_->get_logger(), "TianRoverDriver Plugin (ns = %s) missing <RBWheelJoint>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->RBWheelJointName.c_str());
        else
            this->RBWheelJointName = _sdf->GetElement("RBWheelJoint")->Get<std::string>();
        this->LBWheelJointName = "BogieLB_WheelLB";
        if(!_sdf->HasElement("LBWheelJoint"))
            RCLCPP_WARN(rosnode_->get_logger(), "TianRoverDriver Plugin (ns = %s) missing <LBWheelJoint>, defaults to \"%s\"",
                  this->robot_namespace_.c_str(), this->LBWheelJointName.c_str());
        else
            this->LBWheelJointName = _sdf->GetElement("LBWheelJoint")->Get<std::string>();

        this->RFBogieJointName = "RightMainRocket_BogieRF";
        if(!_sdf->HasElement("RFBogieJoint"))
            RCLCPP_WARN(rosnode_->get_logger(), "TianRoverDriver Plugin (ns = %s) missing <RFBogieJoint>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->RFBogieJointName.c_str());
        else
            this->RFBogieJointName = _sdf->GetElement("RFBogieJoint")->Get<std::string>();
        this->LFBogieJointName = "LeftMainRocket_BogieLF";
        if(!_sdf->HasElement("LFBogieJoint"))
            RCLCPP_WARN(rosnode_->get_logger(), "TianRoverDriver Plugin (ns = %s) missing <LFBogieJoint>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->LFBogieJointName.c_str());
        else
            this->LFBogieJointName = _sdf->GetElement("LFBogieJoint")->Get<std::string>();

        this->RBBogieJointName = "RightViceRocket_BogieRB";
        if(!_sdf->HasElement("RBBogieJoint"))
            RCLCPP_WARN(rosnode_->get_logger(), "TianRoverDriver Plugin (ns = %s) missing <RBBogieJoint>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->RBBogieJointName.c_str());
        else
            this->RBBogieJointName = _sdf->GetElement("RBBogieJoint")->Get<std::string>();
        this->LBBogieJointName = "LeftViceRocket_BogieLB";
        if(!_sdf->HasElement("LBBogieJoint"))
            RCLCPP_WARN(rosnode_->get_logger(), "TianRoverDriver Plugin (ns = %s) missing <LBBogieJoint>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->LBBogieJointName.c_str());
        else
            this->LBBogieJointName = _sdf->GetElement("LBBogieJoint")->Get<std::string>();

        this->right_rocker_joint_name_ = "Body_RightMainRocket";
        if(_sdf->HasElement("rightRockerJoint"))
            this->right_rocker_joint_name_ = _sdf->GetElement("rightRockerJoint")->Get<std::string>();

        this->left_rocker_joint_name_ = "Body_LeftMainRocket";
        if(_sdf->HasElement("leftRockerJoint"))
            this->left_rocker_joint_name_ = _sdf->GetElement("leftRockerJoint")->Get<std::string>();

        this->wheel_separation_ = 0.4;
        if(!_sdf->HasElement("wheelSeparation"))
            RCLCPP_WARN(rosnode_->get_logger(), "TianRoverDriver Plugin (ns = %s) missing <wheelSeparation>, defaults to value from robot_description: %f",
                this->robot_namespace_.c_str(), this->wheel_separation_);
        else
            this->wheel_separation_ = _sdf->GetElement("wheelSeparation")->Get<double>();

        this->front_wheel_separation_ = this->wheel_separation_;
        if(_sdf->HasElement("frontWheelSeparation"))
            this->front_wheel_separation_ = _sdf->GetElement("frontWheelSeparation")->Get<double>();

        this->middle_wheel_separation_ = this->wheel_separation_;
        if(_sdf->HasElement("middleWheelSeparation"))
            this->middle_wheel_separation_ = _sdf->GetElement("middleWheelSeparation")->Get<double>();

        this->rear_wheel_separation_ = this->wheel_separation_;
        if(_sdf->HasElement("rearWheelSeparation"))
            this->rear_wheel_separation_ = _sdf->GetElement("rearWheelSeparation")->Get<double>();

        this->wheel_diameter_ = 0.3;
        if(!_sdf->HasElement("wheelDiameter"))
            RCLCPP_WARN(rosnode_->get_logger(), "TianRoverDriver Plugin (ns = %s) missing <wheelDiameter>, defaults to %f",
                this->robot_namespace_.c_str(), this->wheel_diameter_);
        else
            this->wheel_diameter_ = _sdf->GetElement("wheelDiameter")->Get<double>();

        this->wheel_base_ = 1.55052;
        if(!_sdf->HasElement("wheelBase"))
            RCLCPP_WARN(rosnode_->get_logger(), "TianRoverDriver Plugin (ns = %s) missing <wheelBase>, defaults to %f",
                this->robot_namespace_.c_str(), this->wheel_base_);
        else
            this->wheel_base_ = _sdf->GetElement("wheelBase")->Get<double>();

        this->torque_ = 5.0;
        if(!_sdf->HasElement("torque"))
            RCLCPP_WARN(rosnode_->get_logger(), "TianRoverDriver Plugin (ns = %s) missing <torque>, defaults to %f",
                this->robot_namespace_.c_str(), this->torque_);
        else
            this->torque_ = _sdf->GetElement("torque")->Get<double>();

        this->steering_p_gain_ = 10.0;
        if(_sdf->HasElement("steeringPGain"))
            this->steering_p_gain_ = _sdf->GetElement("steeringPGain")->Get<double>();
        this->steering_i_gain_ = 0.0;
        if(_sdf->HasElement("steeringIGain"))
            this->steering_i_gain_ = _sdf->GetElement("steeringIGain")->Get<double>();
        this->steering_d_gain_ = 0.4;
        if(_sdf->HasElement("steeringDGain"))
            this->steering_d_gain_ = _sdf->GetElement("steeringDGain")->Get<double>();
        this->steering_torque_ = 10.0;
        if(_sdf->HasElement("steeringTorque"))
            this->steering_torque_ = _sdf->GetElement("steeringTorque")->Get<double>();
        this->steering_rate_ = 0.6;
        if(_sdf->HasElement("steeringRate"))
            this->steering_rate_ = _sdf->GetElement("steeringRate")->Get<double>();

        if(_sdf->HasElement("rockerDifferentialPGain"))
            this->rocker_differential_p_gain_ = _sdf->GetElement("rockerDifferentialPGain")->Get<double>();
        if(_sdf->HasElement("rockerDifferentialDGain"))
            this->rocker_differential_d_gain_ = _sdf->GetElement("rockerDifferentialDGain")->Get<double>();
        if(_sdf->HasElement("rockerDifferentialTorque"))
            this->rocker_differential_torque_ = _sdf->GetElement("rockerDifferentialTorque")->Get<double>();
        if(_sdf->HasElement("rockerDifferentialTarget"))
            this->rocker_differential_target_ = _sdf->GetElement("rockerDifferentialTarget")->Get<double>();

        this->command_topic_ = "cmd_vel";
        if(!_sdf->HasElement("commandTopic"))
            RCLCPP_WARN(rosnode_->get_logger(), "TianRoverDriver Plugin (ns = %s) missing <commandTopic>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->command_topic_.c_str());
        else
            this->command_topic_ = _sdf->GetElement("commandTopic")->Get<std::string>();

        this->ackermann_topic_ = "ackermann_cmd";
        if(!_sdf->HasElement("ackermannTopic"))
            RCLCPP_WARN(rosnode_->get_logger(), "TianRoverDriver Plugin (ns = %s) missing <ackermannTopic>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->ackermann_topic_.c_str());
        else
            this->ackermann_topic_ = _sdf->GetElement("ackermannTopic")->Get<std::string>();

        this->rover_motion_mode_topic_ = "rover_motion_mode";
        if(!_sdf->HasElement("roverMotionModeTopic"))
            RCLCPP_WARN(rosnode_->get_logger(), "TianRoverDriver Plugin (ns = %s) missing <roverMotionModeTopic>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->rover_motion_mode_topic_.c_str());
        else
            this->rover_motion_mode_topic_ = _sdf->GetElement("roverMotionModeTopic")->Get<std::string>();

        this->path_topic_ = "path";
        if(!_sdf->HasElement("PathTopic"))
            RCLCPP_WARN(rosnode_->get_logger(), "TianRoverDriver Plugin (ns = %s) missing <odometryTopic>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->path_topic_.c_str());
        else
            this->path_topic_ = _sdf->GetElement("PathTopic")->Get<std::string>();

        this->odometry_topic_ = "odometry";
        if(!_sdf->HasElement("odometryTopic"))
            RCLCPP_WARN(rosnode_->get_logger(), "TianRoverDriver Plugin (ns = %s) missing <odometryTopic>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->odometry_topic_.c_str());
        else
            this->odometry_topic_ = _sdf->GetElement("odometryTopic")->Get<std::string>();

        this->odometry_frame_ = "odom";
        if(!_sdf->HasElement("odometryFrame"))
            RCLCPP_WARN(rosnode_->get_logger(), "TianRoverDriver Plugin (ns = %s) missing <odometryFrame>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->odometry_frame_.c_str());
        else
            this->odometry_frame_ = _sdf->GetElement("odometryFrame")->Get<std::string>();

        this->robot_base_frame_ = "base_footprint";
        if(!_sdf->HasElement("robotBaseFrame"))
            RCLCPP_WARN(rosnode_->get_logger(), "TianRoverDriver Plugin (ns = %s) missing <robotBaseFrame>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->robot_base_frame_.c_str());
        else
            this->robot_base_frame_ = _sdf->GetElement("robotBaseFrame")->Get<std::string>();

        if(this->update_rate_ > 0.0)
            this->update_period_ = 1.0 / this->update_rate_;//update_rate_ = 100,update_period_ = 0.01
        else
            this->update_period_ = 0.0;
        last_update_time_ = this->world->SimTime();

        joints[WRF] = this->parent->GetJoint(RFWheelJointName);
        joints[WLF] = this->parent->GetJoint(LFWheelJointName);
        joints[WRM] = this->parent->GetJoint(RMWheelJointName);
        joints[WLM] = this->parent->GetJoint(LMWheelJointName);
        joints[WRB] = this->parent->GetJoint(RBWheelJointName);
        joints[WLB] = this->parent->GetJoint(LBWheelJointName);
        joints[BRF] = this->parent->GetJoint(this->RFBogieJointName);
        joints[BLF] = this->parent->GetJoint(this->LFBogieJointName);
        joints[BRB] = this->parent->GetJoint(this->RBBogieJointName);
        joints[BLB] = this->parent->GetJoint(this->LBBogieJointName);
        right_rocker_joint_ = this->parent->GetJoint(this->right_rocker_joint_name_);
        left_rocker_joint_ = this->parent->GetJoint(this->left_rocker_joint_name_);

        if(!joints[WLF])
        {
            char error[200];
            snprintf(error, 200,"GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get left front hinge joint named \"%s\"",
                this->robot_namespace_.c_str(), this->LFWheelJointName.c_str());
            gzthrow(error);
        }

        if(!joints[WRF])
        {
            char error[200];
            snprintf(error, 200,"GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get right front hinge joint named \"%s\"",
                this->robot_namespace_.c_str(), this->RFWheelJointName.c_str());
            gzthrow(error);
        }

        if(!joints[WLB])
        {
            char error[200];
            snprintf(error, 200,"GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get left rear hinge joint named \"%s\"",
                this->robot_namespace_.c_str(), this->LBWheelJointName.c_str());
            gzthrow(error);
       }

       if(!joints[WRB])
       {
            char error[200];
            snprintf(error, 200,"GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get right rear hinge joint named \"%s\"",
                this->robot_namespace_.c_str(), this->RBWheelJointName.c_str());
            gzthrow(error);
       }

        if(!joints[BRF] || !joints[BLF] || !joints[BRB] || !joints[BLB])
        {
            gzthrow("TianRoverDriver Plugin couldn't get one or more steering joints");
        }
        if(!right_rocker_joint_ || !left_rocker_joint_)
        {
            gzthrow("TianRoverDriver Plugin couldn't get one or more rocker joints");
        }

        joint_controller_ = this->parent->GetJointController();
        const common::PID steering_pid(
            steering_p_gain_, steering_i_gain_, steering_d_gain_,
            0.0, 0.0, steering_torque_, -steering_torque_);
        for(int joint_index = BRF; joint_index <= BLB; ++joint_index)
        {
            const std::string joint_name = joints[joint_index]->GetScopedName();
            joint_controller_->SetPositionPID(joint_name, steering_pid);
            joint_controller_->SetPositionTarget(joint_name, 0.0);
        }

        #if GAZEBO_MAJOR_VERSION > 2
            joints[WLF]->SetParam("fmax", 0, torque_);
            joints[WRF]->SetParam("fmax", 0, torque_);
            joints[WLB]->SetParam("fmax", 0, torque_);
            joints[WRM]->SetParam("fmax", 0, torque_);
            joints[WLM]->SetParam("fmax", 0, torque_);
            joints[WRB]->SetParam("fmax", 0, torque_);
        #else
            joints[WLF]->SetMaxForce(0, torque_);
            joints[WRF]->SetMaxForce(0, torque_);
            joints[WLB]->SetMaxForce(0, torque_);
            joints[WRB]->SetMaxForce(0, torque_);
        #endif

        transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(rosnode_);
        odometry_publisher_ = rosnode_->create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 10);
        path_publisher_ = rosnode_->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
        path_.header.frame_id = odometry_frame_;
        cmd_vel_subscriber_ = rosnode_->create_subscription<geometry_msgs::msg::Twist>(command_topic_, 1,
            std::bind(&GazeboRosSkidSteerDrive::cmdVelCallback, this, std::placeholders::_1));
        ackermann_subscriber_ = rosnode_->create_subscription<ackermann_msgs::msg::AckermannDrive>(ackermann_topic_, 1,
            std::bind(&GazeboRosSkidSteerDrive::ackermannCallback, this, std::placeholders::_1));
        rover_motion_mode_subscriber_ = rosnode_->create_subscription<std_msgs::msg::String>(rover_motion_mode_topic_, 1,
            std::bind(&GazeboRosSkidSteerDrive::roverMotionModeCallback, this, std::placeholders::_1));
        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&GazeboRosSkidSteerDrive::UpdateChild, this));

    }

    void GazeboRosSkidSteerDrive::UpdateChild()
    {
        common::Time current_time = this->world->SimTime();//这是 Gazebo 仿真时间，等于仿真步长乘以步数
        double seconds_since_last_update = (current_time - last_update_time_).Double();
        if (seconds_since_last_update < 0) {
            last_update_time_ = current_time;
            boost::mutex::scoped_lock scoped_lock(lock);
            last_command_time_ = -1.0;
            setCommandVelocity(0.0, 0.0);
            path_.poses.clear();
            return;
        }
        if(seconds_since_last_update >= update_period_)
        {
            boost::mutex::scoped_lock scoped_lock(lock);
            {
                const double age = rosnode_->now().seconds() - last_command_time_;
                if (last_command_time_ < 0 || age < 0 || age > command_timeout_)
                    setCommandVelocity(0.0, 0.0);
            }
            publishOdometry(seconds_since_last_update);
            getWheelVelocities();
            double alpha_target;
            double beta_target;
            double rear_alpha_target;
            double rear_beta_target;
            {
                alpha_target = alpha_;
                beta_target = beta_;
                rear_alpha_target = rear_alpha_;
                rear_beta_target = rear_beta_;
            }
            const double max_steering_step = steering_rate_ * seconds_since_last_update;
            applied_alpha_ += std::max(-max_steering_step,
                std::min(max_steering_step, alpha_target - applied_alpha_));
            applied_beta_ += std::max(-max_steering_step,
                std::min(max_steering_step, beta_target - applied_beta_));
            applied_rear_alpha_ += std::max(-max_steering_step,
                std::min(max_steering_step, rear_alpha_target - applied_rear_alpha_));
            applied_rear_beta_ += std::max(-max_steering_step,
                std::min(max_steering_step, rear_beta_target - applied_rear_beta_));
            #if GAZEBO_MAJOR_VERSION > 2
                joints[WRF]->SetParam("vel", 0, wheel_speed_[WRF] / (wheel_diameter_ / 2.0));
                joints[WLF]->SetParam("vel", 0, wheel_speed_[WLF] / (wheel_diameter_ / 2.0));
                joints[WRM]->SetParam("vel", 0, wheel_speed_[WRM] / (wheel_diameter_ / 2.0));
                joints[WLM]->SetParam("vel", 0, wheel_speed_[WLM] / (wheel_diameter_ / 2.0));
                joints[WRB]->SetParam("vel", 0, wheel_speed_[WRB] / (wheel_diameter_ / 2.0));
                joints[WLB]->SetParam("vel", 0, wheel_speed_[WLB] / (wheel_diameter_ / 2.0));
            #else
                joints[WRF]->SetVelocity(0, wheel_speed_[WRF] / (wheel_diameter_ / 2.0));
                joints[WLF]->SetVelocity(0, wheel_speed_[WLF] / (wheel_diameter_ / 2.0));
                joints[WRM]->SetVelocity(0, wheel_speed_[WRM] / (wheel_diameter_ / 2.0));
                joints[WLM]->SetVelocity(0, wheel_speed_[WLM] / (wheel_diameter_ / 2.0));
                joints[WRB]->SetVelocity(0, wheel_speed_[WRB] / (wheel_diameter_ / 2.0));
                joints[WLB]->SetVelocity(0, wheel_speed_[WLB] / (wheel_diameter_ / 2.0));
            #endif

            joint_controller_->SetPositionTarget(
                joints[BRF]->GetScopedName(), applied_alpha_);
            joint_controller_->SetPositionTarget(
                joints[BLF]->GetScopedName(), applied_beta_);
            joint_controller_->SetPositionTarget(
                joints[BRB]->GetScopedName(), applied_rear_alpha_);
            joint_controller_->SetPositionTarget(
                joints[BLB]->GetScopedName(), applied_rear_beta_);
            applyRockerDifferential();

            last_update_time_ = current_time;
        }
    }

    void GazeboRosSkidSteerDrive::applyRockerDifferential()
    {
        if(!right_rocker_joint_ || !left_rocker_joint_)
            return;

        const double average_position =
            0.5 * (right_rocker_joint_->Position(0) + left_rocker_joint_->Position(0));
        const double average_velocity =
            0.5 * (right_rocker_joint_->GetVelocity(0) + left_rocker_joint_->GetVelocity(0));
        const double effort = std::max(-rocker_differential_torque_,
            std::min(rocker_differential_torque_,
                -rocker_differential_p_gain_ * (average_position - rocker_differential_target_)
                - rocker_differential_d_gain_ * average_velocity));

        right_rocker_joint_->SetForce(0, effort);
        left_rocker_joint_->SetForce(0, effort);
    }

    void GazeboRosSkidSteerDrive::getWheelVelocities()
    {
        if(std::abs(turn_radius_) > 1.0e-6)
        {
            const double half_wheel_base = wheel_base_ / 2.0;
            const double right_front_y = -front_wheel_separation_ / 2.0;
            const double left_front_y = front_wheel_separation_ / 2.0;
            const double right_middle_y = -middle_wheel_separation_ / 2.0;
            const double left_middle_y = middle_wheel_separation_ / 2.0;
            const double right_rear_y = -rear_wheel_separation_ / 2.0;
            const double left_rear_y = rear_wheel_separation_ / 2.0;
            const double direction = x_ >= 0.0 ? 1.0 : -1.0;

            wheel_speed_[WRF] = direction * wheelLinearSpeed(half_wheel_base, right_front_y);
            wheel_speed_[WLF] = -direction * wheelLinearSpeed(half_wheel_base, left_front_y);
            wheel_speed_[WRM] = direction * wheelLinearSpeed(0.0, right_middle_y);
            wheel_speed_[WLM] = -direction * wheelLinearSpeed(0.0, left_middle_y);
            wheel_speed_[WRB] = direction * wheelLinearSpeed(-half_wheel_base, right_rear_y);
            wheel_speed_[WLB] = -direction * wheelLinearSpeed(-half_wheel_base, left_rear_y);
        }
        else
        {
            if(isCMD)
            {
                ;
            }
            else
            {
                wheel_speed_[WRB] = x_;
                wheel_speed_[WLB]  = -x_;
                wheel_speed_[WRF]= x_;
                wheel_speed_[WLF] = -x_;
                wheel_speed_[WRM]= x_;
                wheel_speed_[WLM] = -x_;
            }
        }

    }

    double GazeboRosSkidSteerDrive::wheelLinearSpeed(double x, double y) const
    {
        const double yaw_rate = x_ / turn_radius_;
        const double radius_x = x;
        const double radius_y = y - turn_radius_;
        return std::abs(yaw_rate) * std::sqrt(radius_x * radius_x + radius_y * radius_y);
    }

    void GazeboRosSkidSteerDrive::publishOdometry(double step_time)
    {
        auto current_time = rosnode_->now();

        ignition::math::Pose3<double> pose = this->parent->WorldPose();

        geometry_msgs::msg::PoseStamped this_pose;
        this_pose.header.frame_id = odometry_frame_;
        this_pose.header.stamp = current_time;
        Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
        T_wb.rotate(Eigen::Quaterniond(pose.Rot().W(),pose.Rot().X(),pose.Rot().Y(),pose.Rot().Z()));
        T_wb.pretranslate(Eigen::Vector3d(pose.Pos().X(),pose.Pos().Y(),pose.Pos().Z()));
        Eigen::Isometry3d T_wp = T_wb*T_bp_;//右乘，T_bp_是在 body 坐标系下 Lidar 的位姿描述
        this_pose.pose.position.x =  T_wp.translation()(0);
        this_pose.pose.position.y =  T_wp.translation()(1);
        this_pose.pose.position.z =  T_wp.translation()(2);
        this_pose.pose.orientation.x = Eigen::Quaterniond(T_wp.rotation()).x();
        this_pose.pose.orientation.y = Eigen::Quaterniond(T_wp.rotation()).y();
        this_pose.pose.orientation.z = Eigen::Quaterniond(T_wp.rotation()).z();
        this_pose.pose.orientation.w = Eigen::Quaterniond(T_wp.rotation()).w();
        path_.header.stamp = current_time;
        path_.poses.push_back(this_pose);
        if (path_.poses.size() > 2000) path_.poses.erase(path_.poses.begin());

        path_publisher_->publish(path_);

        if (broadcast_tf_) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = current_time;
            tf.header.frame_id = odometry_frame_;
            tf.child_frame_id = robot_base_frame_;
            tf.transform.translation.x = pose.Pos().X();
            tf.transform.translation.y = pose.Pos().Y();
            tf.transform.translation.z = pose.Pos().Z();
            tf.transform.rotation.x = pose.Rot().X();
            tf.transform.rotation.y = pose.Rot().Y();
            tf.transform.rotation.z = pose.Rot().Z();
            tf.transform.rotation.w = pose.Rot().W();
            transform_broadcaster_->sendTransform(tf);
        }

        odom_.header.stamp = current_time;
        odom_.header.frame_id = odometry_frame_;
        odom_.child_frame_id = robot_base_frame_;

        odom_.pose.pose.position.x = pose.Pos().X();
        odom_.pose.pose.position.y = pose.Pos().Y();
        odom_.pose.pose.position.z = pose.Pos().Z();
        odom_.pose.pose.orientation.x = pose.Rot().X();
        odom_.pose.pose.orientation.y = pose.Rot().Y();
        odom_.pose.pose.orientation.z = pose.Rot().Z();
        odom_.pose.pose.orientation.w = pose.Rot().W();

        ignition::math::Vector3<double> linear,angular;
        linear = pose.Rot().RotateVectorReverse(this->parent->WorldLinearVel());
        angular = pose.Rot().RotateVectorReverse(this->parent->WorldAngularVel());
        odom_.twist.twist.linear.x = linear.X();
        odom_.twist.twist.linear.y = linear.Y();
        odom_.twist.twist.linear.z = linear.Z();
        odom_.twist.twist.angular.x = angular.X();
        odom_.twist.twist.angular.y = angular.Y();
        odom_.twist.twist.angular.z = angular.Z();

        odometry_publisher_->publish(odom_);
    }
    GZ_REGISTER_MODEL_PLUGIN(GazeboRosSkidSteerDrive)
}
