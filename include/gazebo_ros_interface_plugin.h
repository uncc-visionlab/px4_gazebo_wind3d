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


#ifndef GAZEBO_ROS_INTERFACE_PLUGIN_H
#define GAZEBO_ROS_INTERFACE_PLUGIN_H

// SYSTEM INCLUDES
#include <random>

#include <Eigen/Core>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/msgs/msgs.hh"

//=================== ROS =====================//
#include <mav_msgs/default_topics.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

//============= GAZEBO MSG TYPES ==============//
#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"

#include "Anemometer.pb.h"
#include "Header.pb.h"
#include "Wind.pb.h"

//=============== ROS MSG TYPES ===============//
#include <px4_gazebo_wind3d/Anemometer.h>
#include "common.h"

namespace gazebo {
    static const bool kPrintOnPluginLoad = false;
    static const bool kPrintOnMsgCallback = false;
    static const std::string kConnectGazeboToRosSubtopic = "connect_gazebo_to_ros_subtopic";
    static const std::string kConnectRosToGazeboSubtopic = "connect_ros_to_gazebo_subtopic";
    // typedef's to make life easier
    typedef const boost::shared_ptr<const gz_std_msgs::ConnectGazeboToRosTopic>
    GzConnectGazeboToRosTopicMsgPtr;
    typedef const boost::shared_ptr<const gz_std_msgs::ConnectRosToGazeboTopic>
    GzConnectRosToGazeboTopicMsgPtr;
    typedef const boost::shared_ptr<const physics_msgs::msgs::Wind> GzWindSpeedMsgPtr;
    typedef const boost::shared_ptr<const gz_sensor_msgs::msgs::Anemometer> GzAnemometerMsgPtr;

    /// \brief    ROS interface plugin for Gazebo.
    /// \details  This routes messages to/from Gazebo and ROS. This is used
    ///           so that individual plugins are not ROS dependent.
    ///           This is a WorldPlugin, only one of these is designed to be enabled
    ///           per Gazebo world.

    class GazeboRosInterfacePlugin : public WorldPlugin {
    public:
        GazeboRosInterfacePlugin();
        ~GazeboRosInterfacePlugin();

        void InitializeParams();
        void Publish();

    protected:
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

        /// \brief  	This gets called by the world update start event.
        /// \details	Calculates IMU parameters and then publishes one IMU message.
        void OnUpdate(const common::UpdateInfo&);

    private:
        /// \brief  Provides a way for GzConnectGazeboToRosTopicMsgCallback() to
        ///         connect a Gazebo subscriber to a ROS publisher.
        /// \details
        ///   GazeboMsgT  The type of the message that will be subscribed to the
        ///   Gazebo framework.
        ///   RosMsgT     The type of the message published to the ROS framework.
        template <typename GazeboMsgT, typename RosMsgT>
        void ConnectHelper(void (GazeboRosInterfacePlugin::*fp)(
                const boost::shared_ptr<GazeboMsgT const>&,
                ros::Publisher),
                GazeboRosInterfacePlugin* ptr, std::string gazeboNamespace,
                std::string gazeboTopicName, std::string rosTopicName,
                transport::NodePtr gz_node_handle);

        std::vector<gazebo::transport::NodePtr> nodePtrs_;
        std::vector<gazebo::transport::SubscriberPtr> subscriberPtrs_;

        // std::string namespace_;

        /// \brief  Handle for the Gazebo node.
        transport::NodePtr gz_node_handle_;

        /// \brief  Handle for the ROS node.
        ros::NodeHandle* ros_node_handle_;

        /// \brief  Pointer to the world.
        physics::WorldPtr world_;

        /// \brief  Pointer to the update event connection.
        event::ConnectionPtr updateConnection_;

        // ============================================ //
        // ====== CONNECT GAZEBO TO ROS MESSAGES ====== //
        // ============================================ //

        transport::SubscriberPtr gz_connect_gazebo_to_ros_topic_sub_;
        void GzConnectGazeboToRosTopicMsgCallback(
                GzConnectGazeboToRosTopicMsgPtr& gz_connect_gazebo_to_ros_topic_msg);

        // ============================================ //
        // ====== CONNECT ROS TO GAZEBO MESSAGES ====== //
        // ============================================ //

        transport::SubscriberPtr gz_connect_ros_to_gazebo_topic_sub_;

        /// @brief    Subscribes to the provided ROS topic and publishes on the
        /// provided Gazebo topic (all info contained within the message).
        /// @details  Will create a Gazebo publisher if one doesn't already exist.
        void GzConnectRosToGazeboTopicMsgCallback(
                GzConnectRosToGazeboTopicMsgPtr& gz_connect_ros_to_gazebo_topic_msg);

        /// \brief      Looks if a publisher on the provided topic already exists, and
        ///             returns it.
        ///             If no publisher exists, this method creates one and returns
        ///             that instead.
        /// \warning    Finding an already created publisher is not supported yet!
//        template <typename T>
//        transport::PublisherPtr FindOrMakeGazeboPublisher(std::string topic);

        // ============================================ //
        // ===== HELPER METHODS FOR MSG CONVERSION ==== //
        // ============================================ //

        void ConvertHeaderGzToRos(
                const gz_std_msgs::Header& gz_header,
                std_msgs::Header_<std::allocator<void> >* ros_header);

        void ConvertHeaderRosToGz(
                const std_msgs::Header_<std::allocator<void> >& ros_header,
                gz_std_msgs::Header* gz_header);

        // ============================================ //
        // ===== GAZEBO->ROS CALLBACKS/CONVERTERS ===== //
        // ============================================ //

        // WIND SPEED
        void GzWindSpeedMsgCallback(GzWindSpeedMsgPtr& gz_wind_speed_msg,
                ros::Publisher ros_publisher);
        // ANEMOMETER
        void GzAnemometerMsgCallback(GzAnemometerMsgPtr& gz_anemometer_msg,
                ros::Publisher ros_publisher);   
        px4_gazebo_wind3d::Anemometer ros_anemometer_msg_;


        // ============================================ //
        // ===== ROS->GAZEBO CALLBACKS/CONVERTERS ===== //
        // ============================================ //

        // WIND SPEED
        void RosWindSpeedMsgCallback(
                const geometry_msgs::Vector3Stamped::Ptr& ros_wind_speed_msg_ptr,
                gazebo::transport::PublisherPtr gz_publisher_ptr);

    };

} // namespace gazebo

#endif /* GAZEBO_ROS_INTERFACE_PLUGIN_H */

