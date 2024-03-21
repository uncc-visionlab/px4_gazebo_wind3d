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
#include "gazebo_anemometer_plugin.h"

// USER HEADERS
#include "ConnectGazeboToRosTopic.pb.h"
#include "WindServerRegistration.pb.h"

namespace gazebo {

    GazeboAnemometerPlugin::~GazeboAnemometerPlugin() {
        updateConnection_->~Connection();
    }

    void GazeboAnemometerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        if (kPrintOnPluginLoad) {
            gzdbg << __FUNCTION__ << "() called." << std::endl;
        }

        gzdbg << "_model = " << _model->GetName() << std::endl;

        // Store the pointer to the model and the world.
        model_ = _model;
        world_ = model_->GetWorld();

        //==============================================//
        //========== READ IN PARAMS FROM SDF ===========//
        //==============================================//

        // Use the robot namespace to create the node handle.
        if (_sdf->HasElement("robotNamespace"))
            namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        else
            gzerr << "[gazebo_anemometer_plugin] Please specify a robotNamespace.\n";

        // Get node handle.
        node_handle_ = transport::NodePtr(new transport::Node());

        // Initialize with default namespace (typically /gazebo/default/).
        node_handle_->Init(namespace_);

        if (_sdf->HasElement("linkName"))
            link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
        else
            gzerr << "[gazebo_anemometer_plugin] Please specify a linkName.\n";

        noise_var_.X() = _sdf->GetElement("noiseX")->Get<double>();
        noise_var_.Y() = _sdf->GetElement("noiseY")->Get<double>();
        noise_var_.Z() = _sdf->GetElement("noiseZ")->Get<double>();

        // Get the pointer to the link.
        link_ = model_->GetLink(link_name_);
        if (link_ == NULL)
            gzthrow("[gazebo_anemometer_plugin] Couldn't find specified link \"" << link_name_ << "\".");

        frame_id_ = link_name_;

        getSdfParam<std::string>(_sdf, "windServerRegisterLinkTopic", wind_server_reglink_topic_,
                wind_server_reglink_topic_);
        getSdfParam<std::string>(_sdf, "windServerLinkTopic", wind_server_link_wind_topic_,
                wind_server_link_wind_topic_);

        // Retrieve the rest of the SDF parameters.
        getSdfParam<std::string>(_sdf, "anemometerTopic", anemometer_topic_, kDefaultAnemometerPubTopic);

        // Wind topic publishing rates
        double pub_rate = 2.0;
        getSdfParam<double>(_sdf, "publishRate", pub_rate, pub_rate);
        pub_interval_ = (pub_rate > 0.0) ? 1 / pub_rate : 0.0;

        // Initialize the normal distribution for pressure.
        double mean = 0.0;

        anemometer_velocity_distribution_X_ = std::normal_distribution<double>(mean, sqrt(noise_var_.X()));
        anemometer_velocity_distribution_Y_ = std::normal_distribution<double>(mean, sqrt(noise_var_.Y()));
        anemometer_velocity_distribution_Z_ = std::normal_distribution<double>(mean, sqrt(noise_var_.Z()));

        // Listen to the update event. This event is broadcast every simulation
        // iteration.
        this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GazeboAnemometerPlugin::OnUpdate, this, _1));

        //==============================================//
        //=== POPULATE STATIC PARTS OF PRESSURE MSG ====//
        //==============================================//

        anemometer_message_.mutable_header()->set_frame_id(frame_id_);
    }

    void GazeboAnemometerPlugin::OnUpdate(const common::UpdateInfo& _info) {
        if (kPrintOnUpdates) {
            gzdbg << __FUNCTION__ << "() called." << std::endl;
        }

        if (!pubs_and_subs_created_) {
            CreatePubsAndSubs();
            pubs_and_subs_created_ = true;
        }

        common::Time current_time = world_->SimTime();

        // Get the current geometric height.
        //double height_geometric_m = model_->WorldPose().Pos().Z();

        // Compute the anemometer_reading   
        // actual sensor output = wind velocity + vehicle translational velocity +  velocity due to vehicle rotation + noise
        ignition::math::Vector3d body_velocity_W = link_->WorldLinearVel();
        ignition::math::Vector3d body_velocity_R = link_->WorldAngularVel() * 0;
        ignition::math::Vector3d wind_velocity(0, 0, 0);
        ignition::math::Vector3d anemometer_value = body_velocity_W + body_velocity_R + wind_velocity;

        // Add noise to the measurement.
        anemometer_value.X() += anemometer_velocity_distribution_X_(random_generator_X_);
        anemometer_value.Y() += anemometer_velocity_distribution_Y_(random_generator_Y_);
        anemometer_value.Z() += anemometer_velocity_distribution_Z_(random_generator_Z_);

        // Fill the anemometer message.
        anemometer_message_.mutable_header()->mutable_stamp()->set_sec(
                current_time.sec);
        anemometer_message_.mutable_header()->mutable_stamp()->set_nsec(
                current_time.nsec);
        gazebo::msgs::Vector3d* anemometer_msg = new gazebo::msgs::Vector3d();
        anemometer_msg->set_x(anemometer_value.X());
        anemometer_msg->set_y(anemometer_value.Y());
        anemometer_msg->set_z(anemometer_value.Z());
        anemometer_message_.set_allocated_velocity(anemometer_msg);
        // Publish the anemometer message.
        anemometer_pub_->Publish(anemometer_message_);
    }

    void GazeboAnemometerPlugin::WindSpeedCallback(GzWindSpeedMsgPtr & wind_speed_msg) {
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

    void GazeboAnemometerPlugin::CreatePubsAndSubs() {
        // Gazebo publishers and subscribers

        // ==================================== //
        // ====== WIND SERVER MSG SETUP ======= //
        // ==================================== //
        wind_server_register_pub_ = node_handle_->Advertise<wind3d_msgs::msgs::WindServerRegistration>(
                wind_server_reglink_topic_, 1);
        wind_server_link_wind_msg_sub_ = node_handle_->Subscribe<physics_msgs::msgs::Wind>(wind_server_link_wind_topic_,
                &GazeboAnemometerPlugin::WindSpeedCallback, this);

        // Register this plugin with the world dynamic wind server
        wind3d_msgs::msgs::WindServerRegistration register_msg;
        register_msg.set_link_name(link_name_);
        register_msg.set_model_name(model_->GetName());
        register_msg.set_namespace_(namespace_);
        register_msg.set_link_wind_topic(wind_server_link_wind_topic_);
        wind_server_register_pub_->Publish(register_msg);

        // ================================================= //
        // ====== ANEMOMETER GAZEBO -> ROS MSG SETUP ======= //
        // ================================================= //

        anemometer_pub_ = node_handle_->Advertise<gz_sensor_msgs::msgs::Anemometer>(
                "~/" + namespace_ + "/" + anemometer_topic_, 1);

        // Create temporary "ConnectGazeboToRosTopic" publisher and message.
        gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
                node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
                "~/" + kConnectGazeboToRosSubtopic, 1);
        gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;
        connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                anemometer_topic_);
        connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                anemometer_topic_);
        connect_gazebo_to_ros_topic_msg.set_msgtype(
                gz_std_msgs::ConnectGazeboToRosTopic::ANEMOMETER);
        connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                true);
    }

    GZ_REGISTER_MODEL_PLUGIN(GazeboAnemometerPlugin);

} // namespace gazebo