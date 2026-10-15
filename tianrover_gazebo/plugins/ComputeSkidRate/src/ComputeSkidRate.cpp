/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include <string>
#include <cmath>

#include <stdlib.h>

#include "ComputeSkidRate.h"

namespace gazebo
{
    GZ_REGISTER_MODEL_PLUGIN(GazeboComputeSkidRate);

    GazeboComputeSkidRate::GazeboComputeSkidRate()
    {
        this->seed = 0;
        this->rosnode_ = nullptr;
    }

    GazeboComputeSkidRate::~GazeboComputeSkidRate()
    {
        this->update_connection_.reset();

    }

    void GazeboComputeSkidRate::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        this->world_ = _parent->GetWorld();
        this->model_ = _parent;
        rosnode_ = gazebo_ros::Node::Get(_sdf);

        this->robot_namespace_ = rosnode_->get_namespace();
        if (!_sdf->HasElement("updateRate"))
        {
            RCLCPP_DEBUG(rosnode_->get_logger(), "ComputeSkidRate plugin missing <updateRate>, defaults to 0.0"
        " (as fast as possible)");
            this->update_rate_ = 0;
        }
        else
            this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();

        if (!_sdf->HasElement("WheelRF"))
        {
            RCLCPP_FATAL(rosnode_->get_logger(), "ComputeSkidRate plugin missing <WheelRF>, cannot proceed");
            return;
        }
        else
            this->WheelName_[RF] = _sdf->GetElement("WheelRF")->Get<std::string>();

        if (!_sdf->HasElement("WheelLF"))
        {
            RCLCPP_FATAL(rosnode_->get_logger(), "ComputeSkidRate plugin missing <WheelLF>, cannot proceed");
            return;
        }
        else
            this->WheelName_[LF] = _sdf->GetElement("WheelLF")->Get<std::string>();

        if (!_sdf->HasElement("WheelRM"))
        {
            RCLCPP_FATAL(rosnode_->get_logger(), "ComputeSkidRate plugin missing <WheelRM>, cannot proceed");
            return;
        }
        else
            this->WheelName_[RM] = _sdf->GetElement("WheelRM")->Get<std::string>();

        if (!_sdf->HasElement("WheelLM"))
        {
            RCLCPP_FATAL(rosnode_->get_logger(), "ComputeSkidRate plugin missing <WheelLM>, cannot proceed");
            return;
        }
        else
            this->WheelName_[LM] = _sdf->GetElement("WheelLM")->Get<std::string>();

        if (!_sdf->HasElement("WheelRB"))
        {
            RCLCPP_FATAL(rosnode_->get_logger(), "ComputeSkidRate plugin missing <WheelRB>, cannot proceed");
            return;
        }
        else
            this->WheelName_[RB] = _sdf->GetElement("WheelRB")->Get<std::string>();

        if (!_sdf->HasElement("WheelLB"))
        {
            RCLCPP_FATAL(rosnode_->get_logger(), "ComputeSkidRate plugin missing <WheelLB>, cannot proceed");
            return;
        }
        else
            this->WheelName_[LB] = _sdf->GetElement("WheelLB")->Get<std::string>();

        if (!_sdf->HasElement("WheelRF_RefLink"))
        {
            RCLCPP_FATAL(rosnode_->get_logger(), "ComputeSkidRate plugin missing <WheelRF_RefLink>, cannot proceed");
            return;
        }
        else
            this->WheelRefLinkName_[RF] = _sdf->GetElement("WheelRF_RefLink")->Get<std::string>();
        if (!_sdf->HasElement("WheelLF_RefLink"))
        {
            RCLCPP_FATAL(rosnode_->get_logger(), "ComputeSkidRate plugin missing <WheelLF_RefLink>, cannot proceed");
            return;
        }
        else
            this->WheelRefLinkName_[LF] = _sdf->GetElement("WheelLF_RefLink")->Get<std::string>();
        if (!_sdf->HasElement("WheelRM_RefLink"))
        {
            RCLCPP_FATAL(rosnode_->get_logger(), "ComputeSkidRate plugin missing <WheelRM_RefLink>, cannot proceed");
            return;
        }
        else
            this->WheelRefLinkName_[RM] = _sdf->GetElement("WheelRM_RefLink")->Get<std::string>();
        if (!_sdf->HasElement("WheelLM_RefLink"))
        {
            RCLCPP_FATAL(rosnode_->get_logger(), "ComputeSkidRate plugin missing <WheelLM_RefLink>, cannot proceed");
            return;
        }
        else
            this->WheelRefLinkName_[LM] = _sdf->GetElement("WheelLM_RefLink")->Get<std::string>();
        if (!_sdf->HasElement("WheelRB_RefLink"))
        {
            RCLCPP_FATAL(rosnode_->get_logger(), "ComputeSkidRate plugin missing <WheelRB_RefLink>, cannot proceed");
            return;
        }
        else
            this->WheelRefLinkName_[RB] = _sdf->GetElement("WheelRB_RefLink")->Get<std::string>();
        if (!_sdf->HasElement("WheelLB_RefLink"))
        {
            RCLCPP_FATAL(rosnode_->get_logger(), "ComputeSkidRate plugin missing <WheelLB_RefLink>, cannot proceed");
            return;
        }
        else
            this->WheelRefLinkName_[LB] = _sdf->GetElement("WheelLB_RefLink")->Get<std::string>();
        if (!_sdf->HasElement("Body"))
        {
            RCLCPP_FATAL(rosnode_->get_logger(), "ComputeSkidRate plugin missing <Body>, cannot proceed");
            return;
        }
        else
            this->BodyName_ = _sdf->GetElement("Body")->Get<std::string>();

        if (!_sdf->HasElement("SkidRateTopicName"))
        {
            RCLCPP_FATAL(rosnode_->get_logger(), "ComputeSkidRate plugin missing <SkidRateTopicName>, cannot proceed");
            return;
        }
        else
            this->SkidRateTopicName_ = _sdf->GetElement("SkidRateTopicName")->Get<std::string>();
        if (!_sdf->HasElement("WheelVelPoseTopicName"))
        {
            RCLCPP_FATAL(rosnode_->get_logger(), "ComputeSkidRate plugin missing <WheelVelPoseTopicName>, cannot proceed");
            return;
        }
        else
            this->WheelVelPoseTopicName_ = _sdf->GetElement("WheelVelPoseTopicName")->Get<std::string>();
        if (!_sdf->HasElement("WheelPoseInBodyTopicName"))
        {
            RCLCPP_FATAL(rosnode_->get_logger(), "ComputeSkidRate plugin missing <WheelPoseInBodyTopicName>, cannot proceed");
            return;
        }
        else
            this->WheelPoseInBodyTopicName_ = _sdf->GetElement("WheelPoseInBodyTopicName")->Get<std::string>();

        this->SkidRatePub_ = this->rosnode_->create_publisher<std_msgs::msg::Float64MultiArray>(this->SkidRateTopicName_, 10);
        this->WheelVelPosePub_[RF] = this->rosnode_->create_publisher<geometry_msgs::msg::PoseStamped>(std::string("WRF_")+this->WheelVelPoseTopicName_, 10);
        this->WheelVelPosePub_[LF] = this->rosnode_->create_publisher<geometry_msgs::msg::PoseStamped>(std::string("WLF_")+this->WheelVelPoseTopicName_, 10);
        this->WheelVelPosePub_[RM] = this->rosnode_->create_publisher<geometry_msgs::msg::PoseStamped>(std::string("WRM_")+this->WheelVelPoseTopicName_, 10);
        this->WheelVelPosePub_[LM] = this->rosnode_->create_publisher<geometry_msgs::msg::PoseStamped>(std::string("WLM_")+this->WheelVelPoseTopicName_, 10);
        this->WheelVelPosePub_[RB] = this->rosnode_->create_publisher<geometry_msgs::msg::PoseStamped>(std::string("WRB_")+this->WheelVelPoseTopicName_, 10);
        this->WheelVelPosePub_[LB] = this->rosnode_->create_publisher<geometry_msgs::msg::PoseStamped>(std::string("WLB_")+this->WheelVelPoseTopicName_, 10);
        this->WheelPoseInBodyPub_ = this->rosnode_->create_publisher<geometry_msgs::msg::PoseArray>(this->WheelPoseInBodyTopicName_,10);

        SkidRate_.data.resize(19,0.0);
        memset(&SkidRateBuf_,0,sizeof(SkidRateBuf_));
        WheelPoseInBody_.poses.resize(6);
        WheelPoseInBody_.header.frame_id = _sdf->Get<std::string>("bodyFrame", this->BodyName_).first;


        this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboComputeSkidRate::UpdateChild, this));
    }

    void GazeboComputeSkidRate::ComputeRelativeVel(const std::string &LinkName,\
                                                const std::string &RefLinkName,\
                                                ignition::math::Vector3d &LinkVelocity,\
                                                ignition::math::Vector3d &LinkAngular)
    {
        gazebo::physics::LinkPtr Link;
        gazebo::physics::LinkPtr RefLink;
        Link = this->model_->GetLink(LinkName);
        if(!Link)
        {
            RCLCPP_ERROR(rosnode_->get_logger(), "ComputeSkidRate plugin: RelativeVel '%s' link doesn't exist!\n", LinkName.c_str());
            LinkVelocity = ignition::math::Vector3d::Zero;
            LinkAngular = ignition::math::Vector3d::Zero;
            return;
        }

        LinkVelocity = Link->WorldLinearVel();
        LinkAngular = Link->WorldAngularVel();

        if(RefLinkName != "/world" &&
        RefLinkName != "world" &&
        RefLinkName != "/map" &&
        RefLinkName != "map")
        {
            RefLink = this->model_->GetLink(RefLinkName);
            if(!RefLink)
            {
                RCLCPP_ERROR(rosnode_->get_logger(), "ComputeSkidRate plugin: RelativeVel '%s' Reflink doesn't exist!\n", RefLinkName.c_str());
                LinkVelocity = ignition::math::Vector3d::Zero;
                LinkAngular = ignition::math::Vector3d::Zero;
                return;
            }
            auto RefLinkVelocity = RefLink->WorldLinearVel();
            auto RefLinkAngular = RefLink->WorldAngularVel();
            auto RefLinkPose = RefLink->WorldPose();
            LinkVelocity = RefLinkPose.Rot().RotateVectorReverse(LinkVelocity - RefLinkVelocity);
            LinkAngular = RefLinkPose.Rot().RotateVectorReverse(LinkAngular - RefLinkAngular);
        }
    }

    void GazeboComputeSkidRate::ComputeRelativePose(const std::string &LinkName,\
                                                    const std::string &RefLinkName,\
                                                    ignition::math::Pose3d &LinkPose)
    {
        gazebo::physics::LinkPtr Link;
        gazebo::physics::LinkPtr RefLink;
        Link = this->model_->GetLink(LinkName);
        if(!Link)
        {
            RCLCPP_ERROR(rosnode_->get_logger(), "ComputeSkidRate plugin: RelativePose '%s' link doesn't exist!\n", LinkName.c_str());
            LinkPose = ignition::math::Pose3d::Zero;
            return;
        }

        LinkPose = Link->WorldPose();

        if(RefLinkName != "/world" &&
        RefLinkName != "world" &&
        RefLinkName != "/map" &&
        RefLinkName != "map")
        {
            RefLink = this->model_->GetLink(RefLinkName);
            if(!RefLink)
            {
                RCLCPP_ERROR(rosnode_->get_logger(), "ComputeSkidRate plugin: RelativePose '%s' Reflink doesn't exist!\n", RefLinkName.c_str());
                LinkPose = ignition::math::Pose3d::Zero;
                return;
            }
            auto RefLinkPose = RefLink->WorldPose();

            LinkPose.Pos() = LinkPose.Pos() - RefLinkPose.Pos();
            LinkPose.Pos() = RefLinkPose.Rot().RotateVectorReverse(LinkPose.Pos());
            LinkPose.Rot() = RefLinkPose.Rot().Inverse()*LinkPose.Rot();
        }
    }

    double GazeboComputeSkidRate::ComputeSkidRate(const std::string &LinkName,\
                                                const std::string &RefLinkName,\
                                                WheelIndex wi,\
                                                double &rr,
                                                double &vel_mag)
    {
        ignition::math::Pose3d pose;
        ignition::math::Vector3d vel = ignition::math::Vector3d::Zero;
        ignition::math::Vector3d angular = ignition::math::Vector3d::Zero;
        rr = 0.0;
        vel_mag = 0.0;
        ComputeRelativeVel(LinkName,RefLinkName,vel,angular);
        rr = angular.Y();
        ComputeRelativeVel(LinkName,std::string("world"),vel,angular);
        vel_mag = std::sqrt(vel.X()*vel.X()+vel.Y()*vel.Y()+vel.Z()*vel.Z());
        if( abs(vel_mag)<1.0e-6 || abs(rr)<1.0e-6 )
            return 0;
        double SkidRate = (vel_mag)/(rr*0.15);
        if(SkidRate>2 || SkidRate<-2)
            return 0;

        WheelVelPose_.header.stamp = rosnode_->now();
        WheelVelPose_.header.frame_id = "world";

        if(vel.Length()>0.01)
        {
            ignition::math::Vector3d vx(vel);
            vx.Normalize();
            ComputeRelativePose(LinkName,std::string("world"),pose);
            ignition::math::Matrix3d m(pose.Rot());
            ignition::math::Vector3d vy(m(0,1),m(1,1),m(2,1));
            if(vy.Length() < 1.0e-6)
                return SkidRate;
            vy.Normalize();
            ignition::math::Vector3d vz = vx.Cross(vy);
            if(vz.Length() < 1.0e-6)
                return SkidRate;
            vz.Normalize();
            m.Axes(vx,vy,vz);
            ignition::math::Quaterniond q(m);

            WheelVelPose_.pose.position.x = pose.Pos().X();
            WheelVelPose_.pose.position.y = pose.Pos().Y();
            WheelVelPose_.pose.position.z = pose.Pos().Z();
            WheelVelPose_.pose.orientation.x = q.X();
            WheelVelPose_.pose.orientation.y = q.Y();
            WheelVelPose_.pose.orientation.z = q.Z();
            WheelVelPose_.pose.orientation.w = q.W();
        }

        this->WheelVelPosePub_[wi]->publish(WheelVelPose_);

        return SkidRate;
    }

    void GazeboComputeSkidRate::UpdateChild()
    {
        common::Time cur_time = this->world_->SimTime();

        if ( this->update_rate_ > 0 && (cur_time-this->last_time_).Double() < (1.0/this->update_rate_) )
            return;
        double tmp_dt = cur_time.Double() - this->last_time_.Double();
        if (tmp_dt != 0)
        {
            this->lock.lock();

            double rr,vel;//wheel rotation rate & vel
            SkidRateBuf_[RF] = ComputeSkidRate(this->WheelName_[RF],this->WheelRefLinkName_[RF],RF,rr,vel);
            std::abs(SkidRateBuf_[RF])<1.0e-6?:this->SkidRate_.data[RF] = SkidRateBuf_[RF];
            this->SkidRate_.data[RF+6] = rr;
            this->SkidRate_.data[RF+12] = vel;
            SkidRateBuf_[LF] = ComputeSkidRate(this->WheelName_[LF],this->WheelRefLinkName_[LF],LF,rr,vel);
            std::abs(SkidRateBuf_[LF])<1.0e-6?:this->SkidRate_.data[LF] = SkidRateBuf_[LF];
            this->SkidRate_.data[LF+6] = rr;
            this->SkidRate_.data[LF+12] = vel;
            SkidRateBuf_[RM] = ComputeSkidRate(this->WheelName_[RM],this->WheelRefLinkName_[RM],RM,rr,vel);
            std::abs(SkidRateBuf_[RM])<1.0e-6?:this->SkidRate_.data[RM] = SkidRateBuf_[RM];
            this->SkidRate_.data[RM+6] = rr;
            this->SkidRate_.data[RM+12] = vel;
            SkidRateBuf_[LM] = ComputeSkidRate(this->WheelName_[LM],this->WheelRefLinkName_[LM],LM,rr,vel);
            std::abs(SkidRateBuf_[LM])<1.0e-6?:this->SkidRate_.data[LM] = SkidRateBuf_[LM];
            this->SkidRate_.data[LM+6] = rr;
            this->SkidRate_.data[LM+12] = vel;
            SkidRateBuf_[RB] = ComputeSkidRate(this->WheelName_[RB],this->WheelRefLinkName_[RB],RB,rr,vel);
            std::abs(SkidRateBuf_[RB])<1.0e-6?:this->SkidRate_.data[RB] = SkidRateBuf_[RB];
            this->SkidRate_.data[RB+6] = rr;
            this->SkidRate_.data[RB+12] = vel;
            SkidRateBuf_[LB] = ComputeSkidRate(this->WheelName_[LB],this->WheelRefLinkName_[LB],LB,rr,vel);
            std::abs(SkidRateBuf_[LB])<1.0e-6?:this->SkidRate_.data[LB] = SkidRateBuf_[LB];
            this->SkidRate_.data[LB+6] = rr;
            this->SkidRate_.data[LB+12] = vel;

            common::Time temp = this->world_->SimTime();
            this->SkidRate_.data[18] = ((temp + cur_time).sec + (temp + cur_time).nsec*1.0e-9)/2.0;

            geometry_msgs::msg::Pose this_pose;
            ignition::math::Pose3d that_pose;
            WheelPoseInBody_.header.stamp = rosnode_->now();
            for(int i = 0;i < 6;i++)
            {
                ComputeRelativePose(this->WheelName_[i],this->BodyName_,that_pose);
                this_pose.position.x = that_pose.Pos().X();
                this_pose.position.y = that_pose.Pos().Y();
                this_pose.position.z = that_pose.Pos().Z();
                this_pose.orientation.w = that_pose.Rot().W();
                this_pose.orientation.x = that_pose.Rot().X();
                this_pose.orientation.y = that_pose.Rot().Y();
                this_pose.orientation.z = that_pose.Rot().Z();
                this->WheelPoseInBody_.poses[i] = this_pose;
            }

            this->lock.unlock();

            this->SkidRatePub_->publish(this->SkidRate_);
            this->WheelPoseInBodyPub_->publish(this->WheelPoseInBody_);

            this->last_time_ = cur_time;
        }
    }

}
