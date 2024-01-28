/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 * Copyright 2024 Andrew Willis <arwillis@charlotte.edu> UNC Charlotte
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gazebo_dynamic_wind_plugin.h"
#include "common.h"
#include <fstream>
#include <math.h>

#include "ConnectGazeboToRosTopic.pb.h"
#include "WindServerRegistration.pb.h"

namespace gazebo
{

    GazeboDynamicWindPlugin::~GazeboDynamicWindPlugin() {

    }

    void GazeboDynamicWindPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        //  if (kPrintOnPluginLoad) {
        //    gzdbg << __FUNCTION__ << "() called." << std::endl;
        //  }

        // Store the pointer to the model.
        model_ = _model;
        world_ = model_->GetWorld();

        double wind_gust_start = kDefaultWindGustStart;
        double wind_gust_duration = kDefaultWindGustDuration;

        //==============================================//
        //========== READ IN PARAMS FROM SDF ===========//
        //==============================================//

        if (_sdf->HasElement("robotNamespace"))
            namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        else
            gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";

        // Create Gazebo Node.
        node_handle_ = gazebo::transport::NodePtr(new transport::Node());

        // Initialize with default namespace (typically /gazebo/default/).
        node_handle_->Init(namespace_);

        //        if (_sdf->HasElement("xyzOffset"))
        //            xyz_offset_ = _sdf->GetElement("xyzOffset")->Get<ignition::math::Vector3d >();
        //        else
        //            gzerr << "[gazebo_dynamic_wind_plugin] Please specify a xyzOffset.\n";

        getSdfParam<std::string>(_sdf, "windServerRegisterLinkTopic", wind_server_reglink_topic_,
                wind_server_reglink_topic_);
        getSdfParam<std::string>(_sdf, "windServerLinkTopic", wind_server_link_wind_topic_,
                wind_server_link_wind_topic_);

        getSdfParam<std::string>(_sdf, "frameId", frame_id_, frame_id_);
        getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_);

        link_ = model_->GetLink(link_name_);
        if (link_ == NULL)
            gzthrow("[gazebo_wind_plugin] Couldn't find specified link \"" << link_name_
                << "\".");

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        update_connection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GazeboDynamicWindPlugin::OnUpdate, this, _1));

    }

    // This gets called by the world update start event.

    void GazeboDynamicWindPlugin::OnUpdate(const common::UpdateInfo & _info) {
        if (kPrintOnUpdates) {
            gzdbg << __FUNCTION__ << "() called." << std::endl;
        }

        if (!pubs_and_subs_created_) {
            CreatePubsAndSubs();
            pubs_and_subs_created_ = true;
        }

        // Get the current simulation time.
        common::Time now = world_->SimTime();

        ignition::math::Vector3d wind_velocity(0.0, 0.0, 0.0);

        // Choose user-specified method for calculating wind velocity.
        // Calculate the wind force.
        //        double wind_strength = wind_force_mean_;
        //        ignition::math::Vector3d wind = wind_strength * wind_direction_;
        //        // Apply a force from the constant wind to the link.
        //        link_->AddForceAtRelativePosition(wind, xyz_offset_);
        //
        ignition::math::Vector3d wind(0.0, 0.0, 0.0);
        ignition::math::Vector3d wind_gust(0.0, 0.0, 0.0);
        // Calculate the wind gust force.
        //        if (now >= wind_gust_start_ && now < wind_gust_end_) {
        //            double wind_gust_strength = wind_gust_force_mean_;
        //            wind_gust = wind_gust_strength * wind_gust_direction_;
        //            // Apply a force from the wind gust to the link.
        //            link_->AddForceAtRelativePosition(wind_gust, xyz_offset_);
        //        }

        //wrench_stamped_msg_.mutable_header()->set_frame_id(frame_id_);
        //wrench_stamped_msg_.mutable_header()->mutable_stamp()->set_sec(now.sec);
        //wrench_stamped_msg_.mutable_header()->mutable_stamp()->set_nsec(now.nsec);

        //wrench_stamped_msg_.mutable_wrench()->mutable_force()->set_x(wind.X() +
        //        wind_gust.X());
        //wrench_stamped_msg_.mutable_wrench()->mutable_force()->set_y(wind.Y() +
        //        wind_gust.Y());
        //wrench_stamped_msg_.mutable_wrench()->mutable_force()->set_z(wind.Z() +
        //        wind_gust.Z());

        // No torque due to wind, set x,y and z to 0.
        //wrench_stamped_msg_.mutable_wrench()->mutable_torque()->set_x(0);
        //wrench_stamped_msg_.mutable_wrench()->mutable_torque()->set_y(0);
        //wrench_stamped_msg_.mutable_wrench()->mutable_torque()->set_z(0);

        //wind_force_pub_->Publish(wrench_stamped_msg_);
    }

    void GazeboDynamicWindPlugin::WindSpeedCallback(GzWindSpeedMsgPtr & wind_speed_msg) {
        //if (kPrintOnMsgCallback) {
        //  gzdbg << __FUNCTION__ << "() called." << std::endl;
        //}
        //gzdbg << __FUNCTION__ << "() called." << std::endl;

        // TODO(burrimi): Transform velocity to world frame if frame_id is set to
        // something else.
        wind_speed_W_.X() = wind_speed_msg->velocity().x();
        wind_speed_W_.Y() = wind_speed_msg->velocity().y();
        wind_speed_W_.Z() = wind_speed_msg->velocity().z();
    }

    void GazeboDynamicWindPlugin::CreatePubsAndSubs() {
        // Create temporary "ConnectGazeboToRosTopic" publisher and message.
        gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
                node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
                "~/" + kConnectGazeboToRosSubtopic, 1);

        gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;
        wind_server_register_pub_ = node_handle_->Advertise<wind3d_msgs::msgs::WindServerRegistration>(
                wind_server_reglink_topic_, 1);
        wind_server_link_wind_msg_sub_ = node_handle_->Subscribe<physics_msgs::msgs::Wind>(wind_server_link_wind_topic_,
                &GazeboDynamicWindPlugin::WindSpeedCallback, this);

        // Register this plugin with the world dynamic wind server
        wind3d_msgs::msgs::WindServerRegistration register_msg;
        register_msg.set_link_name(link_name_);
        register_msg.set_model_name(model_->GetName());
        register_msg.set_namespace_(namespace_);
        register_msg.set_link_wind_topic(wind_server_link_wind_topic_);        
        wind_server_register_pub_->Publish(register_msg);

        // ============================================ //
        // ========= WRENCH STAMPED MSG SETUP ========= //
        // ============================================ //
        //wind_force_pub_ = node_handle_->Advertise<gz_geometry_msgs::WrenchStamped>(
        //        "~/" + namespace_ + "/" + wind_force_pub_topic_, 1);

        // connect_gazebo_to_ros_topic_msg.set_gazebo_namespace(namespace_);
        //connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
        //        wind_force_pub_topic_);
        //connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
        //        wind_force_pub_topic_);
        //connect_gazebo_to_ros_topic_msg.set_msgtype(
        //        gz_std_msgs::ConnectGazeboToRosTopic::WRENCH_STAMPED);
        //connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
        //        true);
        //
        // ============================================ //
        // ========== WIND SPEED MSG SETUP ============ //
        // ============================================ //
        //  wind_speed_pub_ = node_handle_->Advertise<gz_mav_msgs::WindSpeed>(
        //      "~/" + namespace_ + "/" + wind_speed_pub_topic_, 1);
        //
        //  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
        //                                                   wind_speed_pub_topic_);
        //  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
        //                                                wind_speed_pub_topic_);
        //  connect_gazebo_to_ros_topic_msg.set_msgtype(
        //      gz_std_msgs::ConnectGazeboToRosTopic::WIND_SPEED);
        //  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
        //                                           true);
    }

    GZ_REGISTER_MODEL_PLUGIN(GazeboDynamicWindPlugin);

} // namespace gazebo