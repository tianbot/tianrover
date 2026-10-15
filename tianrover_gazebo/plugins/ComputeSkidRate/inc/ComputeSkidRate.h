/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GAZEBO_ROS_P3D_HH
#define GAZEBO_ROS_P3D_HH

#include <string>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
    enum WheelIndex
    {
        RF=0,
        LF=1,
        RM=2,
        LM=3,
        RB=4,
        LB=5,
    };

    class GazeboComputeSkidRate : public ModelPlugin
    {
    public:
        GazeboComputeSkidRate();
        virtual ~GazeboComputeSkidRate();
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    protected:
        virtual void UpdateChild();
    private:
        void ComputeRelativeVel(const std::string &LinkName,\
                                const std::string &RefLinkName,\
                                ignition::math::Vector3d &LinkVelocity,\
                                ignition::math::Vector3d &LinkAngular);
        void ComputeRelativePose(const std::string &LinkName,\
                                 const std::string &RefLinkName,\
                                 ignition::math::Pose3d &Pose);
        double ComputeSkidRate(const std::string &LinkName,\
                               const std::string &RefLinkName,\
                               WheelIndex wi,\
                               double &rr,
                               double &vel_mag);

        gazebo_ros::Node::SharedPtr rosnode_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr SkidRatePub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr WheelVelPosePub_[6];
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr WheelPoseInBodyPub_;
        std::string SkidRateTopicName_;
        std::string WheelVelPoseTopicName_;
        std::string WheelPoseInBodyTopicName_;

        physics::WorldPtr world_;
        physics::ModelPtr model_;

        std_msgs::msg::Float64MultiArray SkidRate_;
        double SkidRateBuf_[6];
        geometry_msgs::msg::PoseStamped WheelVelPose_;
        geometry_msgs::msg::PoseArray WheelPoseInBody_;

        std::string WheelName_[6];
        std::string WheelRefLinkName_[6];
        std::string BodyName_;

        boost::mutex lock;
        common::Time last_time_;

        double update_rate_;
        std::string robot_namespace_;


        event::ConnectionPtr update_connection_;
        unsigned int seed;

  };
}
#endif
