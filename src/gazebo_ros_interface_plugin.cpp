/*
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

// MODULE
#include "gazebo_ros_interface_plugin.h"

// SYSTEM
#include <stdio.h>
#include <chrono>
#include <cmath>
#include <iostream>

// 3RD PARTY
#include <std_msgs/Header.h>
#include <boost/bind.hpp>

namespace gazebo
{

    GazeboRosInterfacePlugin::GazeboRosInterfacePlugin()
            : WorldPlugin(), gz_node_handle_(0), ros_node_handle_(0) {
    }

    GazeboRosInterfacePlugin::~GazeboRosInterfacePlugin() {

        // Shutdown and delete ROS node handle
        if (ros_node_handle_) {
            ros_node_handle_->shutdown();
            delete ros_node_handle_;
        }
    }

    void GazeboRosInterfacePlugin::Load(physics::WorldPtr _world,
            sdf::ElementPtr _sdf) {
        if (kPrintOnPluginLoad) {
            gzdbg << __FUNCTION__ << "() called." << std::endl;
        }

        /// \brief    Store the pointer to the model.
        world_ = _world;

        // namespace_.clear();

        //==============================================//
        //========== READ IN PARAMS FROM SDF ===========//
        //==============================================//

        /*if (_sdf->HasElement("robotNamespace"))
          namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        else
          gzerr << "Please specify a robotNamespace.\n";
        gzdbg << "namespace_ = \"" << namespace_ << "\"." << std::endl;*/

        // Get Gazebo node handle
        gz_node_handle_ = transport::NodePtr(new transport::Node());
        // gz_node_handle_->Init(namespace_);
        gz_node_handle_->Init();

        // Get ROS node handle
        // ros_node_handle_ = new ros::NodeHandle(namespace_);
        ros_node_handle_ = new ros::NodeHandle();

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GazeboRosInterfacePlugin::OnUpdate, this, _1));

        // ============================================ //
        // === CONNECT GAZEBO TO ROS MESSAGES SETUP === //
        // ============================================ //

        gz_connect_gazebo_to_ros_topic_sub_ = gz_node_handle_->Subscribe(
                "~/" + kConnectGazeboToRosSubtopic,
                &GazeboRosInterfacePlugin::GzConnectGazeboToRosTopicMsgCallback, this);

        // ============================================ //
        // === CONNECT ROS TO GAZEBO MESSAGES SETUP === //
        // ============================================ //

        gz_connect_ros_to_gazebo_topic_sub_ = gz_node_handle_->Subscribe(
                "~/" + kConnectRosToGazeboSubtopic,
                &GazeboRosInterfacePlugin::GzConnectRosToGazeboTopicMsgCallback, this);

        // ============================================ //
        // ===== BROADCAST TRANSFORM MESSAGE SETUP ==== //
        // ============================================ //

//        gz_broadcast_transform_sub_ = gz_node_handle_->Subscribe(
//                "~/" + kBroadcastTransformSubtopic,
//                &GazeboRosInterfacePlugin::GzBroadcastTransformMsgCallback, this);
    }

    void GazeboRosInterfacePlugin::OnUpdate(const common::UpdateInfo & _info) {
        // Do nothing
        // This plugins actions are all executed through message callbacks.
    }

    /// \brief      A helper class that provides storage for additional parameters
    ///             that are inserted into the callback.
    /// \details
    ///   GazeboMsgT  The type of the message that will be subscribed to the Gazebo
    ///   framework.

    template <typename GazeboMsgT>
            struct ConnectHelperStorage {
        /// \brief    Pointer to the ROS interface plugin class.
        GazeboRosInterfacePlugin* ptr;

        /// \brief    Function pointer to the subscriber callback with additional
        /// parameters.
        void (GazeboRosInterfacePlugin::*fp)(
                const boost::shared_ptr<GazeboMsgT const>&, ros::Publisher ros_publisher);

        /// \brief    The ROS publisher that is passed into the modified callback.
        ros::Publisher ros_publisher;

        /// \brief    This is what gets passed into the Gazebo Subscribe method as a
        ///           callback, and hence can only
        ///           have one parameter (note boost::bind() does not work with the
        ///           current Gazebo Subscribe() definitions).

        void callback(const boost::shared_ptr<GazeboMsgT const>& msg_ptr) {
            (ptr->*fp)(msg_ptr, ros_publisher);
        }
    };

    template <typename GazeboMsgT, typename RosMsgT>
            void GazeboRosInterfacePlugin::ConnectHelper(
            void (GazeboRosInterfacePlugin::*fp)(
            const boost::shared_ptr<GazeboMsgT const>&, ros::Publisher),
            GazeboRosInterfacePlugin* ptr, std::string gazeboNamespace,
            std::string gazeboTopicName, std::string rosTopicName,
            transport::NodePtr gz_node_handle) {
        // One map will be created for each Gazebo message type
        static std::map<std::string, ConnectHelperStorage<GazeboMsgT> > callback_map;

        // Create ROS publisher
        ros::Publisher ros_publisher =
                ros_node_handle_->advertise<RosMsgT>(rosTopicName, 1);

        auto callback_entry = callback_map.emplace(
                gazeboTopicName,
                ConnectHelperStorage<GazeboMsgT>{ptr, fp, ros_publisher});

        // Check if element was already present
        if (!callback_entry.second)
            gzerr << "Tried to add element to map but the gazebo topic name was "
                "already present in map."
                << std::endl;

        // Create subscriber
        gazebo::transport::SubscriberPtr subscriberPtr;
        subscriberPtr = gz_node_handle->Subscribe(
                gazeboTopicName, &ConnectHelperStorage<GazeboMsgT>::callback,
                &callback_entry.first->second);

        // Save a reference to the subscriber pointer so subscriber
        // won't be deleted.
        subscriberPtrs_.push_back(subscriberPtr);
    }

    void GazeboRosInterfacePlugin::GzConnectGazeboToRosTopicMsgCallback(
            GzConnectGazeboToRosTopicMsgPtr & gz_connect_gazebo_to_ros_topic_msg) {
        if (kPrintOnMsgCallback) {
            gzdbg << __FUNCTION__ << "() called." << std::endl;
        }

        const std::string gazeboNamespace =
                ""; // gz_connect_gazebo_to_ros_topic_msg->gazebo_namespace();
        const std::string gazeboTopicName =
                gz_connect_gazebo_to_ros_topic_msg->gazebo_topic();
        const std::string rosTopicName =
                gz_connect_gazebo_to_ros_topic_msg->ros_topic();

        gzdbg << "Connecting Gazebo topic \"" << gazeboTopicName
                << "\" to ROS topic \"" << rosTopicName << "\"." << std::endl;

        switch (gz_connect_gazebo_to_ros_topic_msg->msgtype()) {

            case gz_std_msgs::ConnectGazeboToRosTopic::ANEMOMETER:
                ConnectHelper<gz_sensor_msgs::msgs::Anemometer,
                        px4_gazebo_wind3d::Anemometer>(
                        &GazeboRosInterfacePlugin::GzAnemometerMsgCallback, this,
                        gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_);
                break;
            default:
                gzthrow("ConnectGazeboToRosTopic message type with enum val = "
                        << gz_connect_gazebo_to_ros_topic_msg->msgtype()
                        << " is not supported by GazeboRosInterfacePlugin.");
        }

        gzdbg << __FUNCTION__ << "() finished." << std::endl;
    }

    void GazeboRosInterfacePlugin::GzConnectRosToGazeboTopicMsgCallback(
            GzConnectRosToGazeboTopicMsgPtr & gz_connect_ros_to_gazebo_topic_msg) {
        if (kPrintOnMsgCallback) {
            gzdbg << __FUNCTION__ << "() called." << std::endl;
        }

        static std::vector<ros::Subscriber> ros_subscribers;

        switch (gz_connect_ros_to_gazebo_topic_msg->msgtype()) {
            default:
            {
                gzthrow("ConnectRosToGazeboTopic message type with enum val = "
                        << gz_connect_ros_to_gazebo_topic_msg->msgtype()
                        << " is not supported by GazeboRosInterfacePlugin.");
            }
        }
    }

    //===========================================================================//
    //==================== HELPER METHODS FOR MSG CONVERSION ====================//
    //===========================================================================//

    void GazeboRosInterfacePlugin::ConvertHeaderGzToRos(
            const gz_std_msgs::Header& gz_header,
            std_msgs::Header_<std::allocator<void> >* ros_header) {
        ros_header->stamp.sec = gz_header.stamp().sec();
        ros_header->stamp.nsec = gz_header.stamp().nsec();
        ros_header->frame_id = gz_header.frame_id();
    }

    void GazeboRosInterfacePlugin::ConvertHeaderRosToGz(
            const std_msgs::Header_<std::allocator<void> >& ros_header,
            gz_std_msgs::Header * gz_header) {
        gz_header->mutable_stamp()->set_sec(ros_header.stamp.sec);
        gz_header->mutable_stamp()->set_nsec(ros_header.stamp.nsec);
        gz_header->set_frame_id(ros_header.frame_id);
    }

    //===========================================================================//
    //================ GAZEBO -> ROS MSG CALLBACKS/CONVERTERS ===================//
    //===========================================================================//


    void GazeboRosInterfacePlugin::GzAnemometerMsgCallback(
            GzAnemometerMsgPtr& gz_anemometer_msg,
            ros::Publisher ros_publisher) {
        // ============================================ //
        // =================== HEADER ================= //
        // ============================================ //
        ConvertHeaderGzToRos(gz_anemometer_msg->header(),
                &ros_anemometer_msg_.header);    
        // ============================================ //
        // ================== VELOCITY ================ //
        // ============================================ //
        ros_anemometer_msg_.velocity.x =
                gz_anemometer_msg->velocity().x();
        ros_anemometer_msg_.velocity.y =
                gz_anemometer_msg->velocity().y();
        ros_anemometer_msg_.velocity.z =
                gz_anemometer_msg->velocity().z();
        ros_publisher.publish(ros_anemometer_msg_);        
    }
    
    void GazeboRosInterfacePlugin::GzWindSpeedMsgCallback(
            GzWindSpeedMsgPtr& gz_wind_speed_msg,
            ros::Publisher ros_publisher) {
        // ============================================ //
        // =================== HEADER ================= //
        // ============================================ //
//        ConvertHeaderGzToRos(gz_wind_speed_msg->header(),
//                &ros_wind_speed_msg_.header);

        // ============================================ //
        // ================== VELOCITY ================ //
        // ============================================ //
//        ros_wind_speed_msg_.velocity.x =
//                gz_wind_speed_msg->velocity().x();
//        ros_wind_speed_msg_.velocity.y =
//                gz_wind_speed_msg->velocity().y();
//        ros_wind_speed_msg_.velocity.z =
//                gz_wind_speed_msg->velocity().z();
//        ros_publisher.publish(ros_wind_speed_msg_);
    }
    //===========================================================================//
    //================ ROS -> GAZEBO MSG CALLBACKS/CONVERTERS ===================//
    //===========================================================================//


    void GazeboRosInterfacePlugin::RosWindSpeedMsgCallback(
            const geometry_msgs::Vector3Stamped::Ptr& ros_wind_speed_msg_ptr,
            gazebo::transport::PublisherPtr gz_publisher_ptr) {
        // Convert ROS message to Gazebo message

//        gz_mav_msgs::WindSpeed gz_wind_speed_msg;
//
//        ConvertHeaderRosToGz(ros_wind_speed_msg_ptr->header,
//                gz_wind_speed_msg.mutable_header());
//
//        gz_wind_speed_msg.mutable_velocity()->set_x(
//                ros_wind_speed_msg_ptr->velocity.x);
//        gz_wind_speed_msg.mutable_velocity()->set_y(
//                ros_wind_speed_msg_ptr->velocity.y);
//        gz_wind_speed_msg.mutable_velocity()->set_z(
//                ros_wind_speed_msg_ptr->velocity.z);
//
//        // Publish to Gazebo
//        gz_publisher_ptr->Publish(gz_wind_speed_msg);
    }

    GZ_REGISTER_WORLD_PLUGIN(GazeboRosInterfacePlugin);

} // namespace gazebo