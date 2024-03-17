/*
 * Copyright 2024 Andrew Willis, UNC Charlotte
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

// MODULE HEADER
#include <gazebo-11/gazebo/rendering/ArrowVisual.hh>
#include <c++/9/bits/shared_ptr.h>
#include <ros/node_handle.h>

#include "gazebo_wind3d_visual_plugin.h"

// USER HEADERS
#include "ConnectGazeboToRosTopic.pb.h"
#include "WindServerRegistration.pb.h"

namespace gazebo
{
    namespace rendering
            {
        GazeboWind3DVisualPlugin::~GazeboWind3DVisualPlugin()
        {
            updateConnection_->~Connection();
        }

        void GazeboWind3DVisualPlugin::Load(VisualPtr _parent, sdf::ElementPtr _sdf)
        {
            if (kPrintOnPluginLoad) {
                gzdbg << __FUNCTION__ << "() called." << std::endl;
            }

            //gzdbg << "_model = " << _model->GetName() << std::endl;

            // Store the pointer to the model and the world.
            //model_ = _model;
            //world_ = model_->GetWorld();
            visual_ = _parent;
            scene_ = _parent->GetScene();
            //visual_namespace_ = "visual/";

            //==============================================//
            //========== READ IN PARAMS FROM SDF ===========//
            //==============================================//

            // Use the robot namespace to create the node handle.
            //            if (_sdf->HasElement("robotNamespace"))
            //                namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
            //            else
            //                gzerr << __FUNCTION__ << " Please specify a robotNamespace.\n";
            //namespace_ = "visual/";
            namespace_ = "";

            // Get node handle.
            node_handle_ = transport::NodePtr(new transport::Node());

            // Initialize with default namespace (typically /gazebo/default/).
            node_handle_->Init(namespace_);

            //            if (_sdf->HasElement("linkName"))
            //                link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
            //            else
            //                gzerr << __FUNCTION__ << "Please specify a linkName.\n";

            // Get the pointer to the link.
            //link_ = model_->GetLink(link_name_);
            //if (link_ == NULL)
            //    gzthrow("[gazebo_anemometer_plugin] Couldn't find specified link \"" << link_name_ << "\".");

            //            getSdfParam<std::string>(_sdf, "windServerRegisterLinkTopic", wind_server_reglink_topic_,
            //            wind_server_reglink_topic_);
            //            getSdfParam<std::string>(_sdf, "windServerLinkTopic", wind_server_link_wind_topic_,
            //            wind_server_link_wind_topic_);
            //ArrowVisualPtr vis(new ArrowVisual("arrow", _parent));
            gzdbg << "Parent visual is " << _parent << std::endl;
            //int rand = rand()%1000 + 1
            const std::string CHARACTERS = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
            std::random_device random_device;
            std::mt19937 generator(random_device());
            std::uniform_int_distribution<> distribution(0, CHARACTERS.size() - 1);
            std::string random_string = "arrow_";
            int NUM_RANDC = 5;
            for (std::size_t i = 0; i < NUM_RANDC; ++i) {
                random_string += CHARACTERS[distribution(generator)];
            }
            arrow_ptr.reset(new ArrowVisual(random_string, _parent));
            //arrow_ptr.reset(new ArrowVisual("arrow", scene_->WorldVisual()));            
            arrow_ptr->Load();
            arrow_ptr->SetScale(ignition::math::Vector3d(100, 100, 100));
            arrow_ptr->SetPose(ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
            arrow_ptr->SetVisibilityFlags(GZ_VISIBILITY_GUI);
            //ArrowVisualPtr
            //visual_->AttachVisual(std::make_shared<ArrowVisual>(ArrowVisual("arrow", scene_->WorldVisual())));
            //visual_->AttachVisual(std::make_shared<ArrowVisual>(ArrowVisual("arrow", _parent)));
            // Listen to the update event. This event is broadcast every simulation
            // iteration.
            updateConnection_ = event::Events::ConnectPreRender(
            boost::bind(&GazeboWind3DVisualPlugin::OnUpdate, this));
            gzdbg << __FUNCTION__ << "() constructed." << std::endl;
        }

        void GazeboWind3DVisualPlugin::OnUpdate()
        {
            if (kPrintOnUpdates) {
                gzdbg << __FUNCTION__ << "() called." << std::endl;
            }

            if (!pubs_and_subs_created_) {
                //                CreatePubsAndSubs();
                pubs_and_subs_created_ = true;
                //                arrow_ptr->SetVisible(true);
                visual_->AttachVisual(arrow_ptr);
                //scene_->WorldVisual()->AttachVisual(arrow_ptr);
                arrow_ptr->SetMaterial("Gazebo/Yellow");
                arrow_ptr->SetVisibilityFlags(GZ_VISIBILITY_GUI);
            }
            arrow_ptr->ShowHead(true);
            arrow_ptr->ShowBoundingBox();
            arrow_ptr->ShowShaft(true);
            arrow_ptr->SetScale(ignition::math::Vector3d(600, 600, 600));
            arrow_ptr->SetPose(ignition::math::Pose3d(0, 0, 1, 0, 0, 0));
            //common::Time current_time = world_->SimTime();

            // Get the current geometric height.
            //double height_geometric_m = model_->WorldPose().Pos().Z();

            // Compute the anemometer_reading   
            // actual sensor output = wind velocity + vehicle translational velocity +  velocity due to vehicle rotation + noise
            //ignition::math::Vector3d body_velocity_W = link_->WorldLinearVel();
            //ignition::math::Vector3d body_velocity_R = link_->WorldAngularVel() * 0;
            //ignition::math::Vector3d wind_velocity(0, 0, 0);
            //ignition::math::Vector3d anemometer_value = body_velocity_W + body_velocity_R + wind_velocity;
        }

        void GazeboWind3DVisualPlugin::WindSpeedCallback(GzWindSpeedMsgPtr & wind_speed_msg)
        {
            //if (kPrintOnMsgCallback) {
            //  gzdbg << __FUNCTION__ << "() called." << std::endl;
            //}
            //gzdbg << __FUNCTION__ << "() called." << std::endl;

            //wind_speed_W_.X() = wind_speed_msg->velocity().x();
            //wind_speed_W_.Y() = wind_speed_msg->velocity().y();
            //wind_speed_W_.Z() = wind_speed_msg->velocity().z();
        }

        void GazeboWind3DVisualPlugin::CreatePubsAndSubs()
        {
            // Gazebo publishers and subscribers

            // ==================================== //
            // ====== WIND SERVER MSG SETUP ======= //
            // ==================================== //
            //            wind_server_register_pub_ = node_handle_->Advertise<wind3d_msgs::msgs::WindServerRegistration>(
            //            wind_server_reglink_topic_, 1);
            //            wind_server_link_wind_msg_sub_ = node_handle_->Subscribe<physics_msgs::msgs::Wind>(wind_server_link_wind_topic_,
            //            &GazeboWind3DVisualPlugin::WindSpeedCallback, this);
            //
            //            // Register this plugin with the world dynamic wind server
            //            wind3d_msgs::msgs::WindServerRegistration register_msg;
            //            register_msg.set_link_name(link_name_);
            //            register_msg.set_model_name(model_->GetName());
            //            register_msg.set_namespace_(namespace_);
            //            register_msg.set_link_wind_topic(wind_server_link_wind_topic_);
            //            wind_server_register_pub_->Publish(register_msg);

            // ================================================= //
            // ====== ANEMOMETER GAZEBO -> ROS MSG SETUP ======= //
            // ================================================= //

            //        anemometer_pub_ = node_handle_->Advertise<gz_sensor_msgs::msgs::Anemometer>(
            //                "~/" + namespace_ + "/" + anemometer_topic_, 1);

            // Create temporary "ConnectGazeboToRosTopic" publisher and message.
            //        gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
            //                node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
            //                "~/" + kConnectGazeboToRosSubtopic, 1);
            //        gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;
            //        connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
            //                anemometer_topic_);
            //        connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
            //                anemometer_topic_);
            //        connect_gazebo_to_ros_topic_msg.set_msgtype(
            //                gz_std_msgs::ConnectGazeboToRosTopic::ANEMOMETER);
            //        connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
            //                true);
        }

        GZ_REGISTER_VISUAL_PLUGIN(GazeboWind3DVisualPlugin);}
} // namespace gazebo