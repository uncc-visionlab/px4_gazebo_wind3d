/*
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

#ifndef GAZEBO_ANEMOMETER_PLUGIN_H
#define GAZEBO_ANEMOMETER_PLUGIN_H

#include <random>

#include <glog/logging.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <mav_msgs/default_topics.h>  // This comes from the mav_comm repo

#include "Anemometer.pb.h"
#include "ConnectGazeboToRosTopic.pb.h"
#include "Wind.pb.h"

#include "common.h"

namespace gazebo {
    // Constants
    static const bool kPrintOnPluginLoad = false;
    static const bool kPrintOnUpdates = false;
    static const std::string kDefaultNamespace = "";
    static const std::string kDefaultFrameId = "world";
    static const std::string kDefaultWindServerRegisterTopic = "/gazebo/default/wind3d_register_link";
    static const std::string kDefaultWindSpeedTopic = mav_msgs::default_topics::WIND_SPEED;
    static const std::string kDefaultLinkName = "base_link";

    // Default values
    static const std::string kDefaultAnemometerPubTopic = "anemometer";

    static const std::string kConnectGazeboToRosSubtopic = "connect_gazebo_to_ros_subtopic";
    static const std::string kConnectRosToGazeboSubtopic = "connect_ros_to_gazebo_subtopic";

    typedef const boost::shared_ptr<const physics_msgs::msgs::Wind>& GzWindSpeedMsgPtr;
    
    class GazeboAnemometerPlugin : public ModelPlugin {
    public:
        /// \brief    Constructor.

        GazeboAnemometerPlugin()
        : ModelPlugin(),
        namespace_(kDefaultNamespace),
        wind_server_reglink_topic_(kDefaultWindServerRegisterTopic),
        wind_server_link_wind_topic_(kDefaultNamespace+"/"+kDefaultLinkName+"/"+kDefaultWindSpeedTopic),
        frame_id_(kDefaultFrameId),
        link_name_(kDefaultLinkName),
        noise_var_(0, 0, 0),
        pub_interval_(0.5),
        node_handle_(nullptr),                
        pubs_and_subs_created_(false) {
        }

        /// \brief    Destructor.
        virtual ~GazeboAnemometerPlugin();

        //        typedef std::normal_distribution<> NormalDistribution;

    protected:
        /// \brief    Called when the plugin is first created, and after the world
        ///           has been loaded. This function should not be blocking.
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        /// \brief  	This gets called by the world update start event.
        void OnUpdate(const common::UpdateInfo&);

    private:
        /// \brief    Flag that is set to true once CreatePubsAndSubs() is called, used
        ///           to prevent CreatePubsAndSubs() from be called on every OnUpdate().
        bool pubs_and_subs_created_;

        /// \brief    Creates all required publishers and subscribers, incl. routing of messages to/from ROS if required.
        /// \details  Call this once the first time OnUpdate() is called (can't
        ///           be called from Load() because there is no guarantee GazeboRosInterfacePlugin has
        ///           has loaded and listening to ConnectGazeboToRosTopic and ConnectRosToGazeboTopic messages).
        void CreatePubsAndSubs();

        void WindSpeedCallback(GzWindSpeedMsgPtr& wind_speed_msg);
        
        /// \brief    Handle for the Gazebo node.
        gazebo::transport::NodePtr node_handle_;

        double pub_interval_;

        /// \brief    Anemometer plugin publishers and subscribers
        gazebo::transport::PublisherPtr wind_server_register_pub_;
        gazebo::transport::SubscriberPtr wind_server_link_wind_msg_sub_;
        gazebo::transport::PublisherPtr anemometer_pub_;

        /// \brief    Transport namespace.
        std::string namespace_;

        /// \brief    Frame ID for anemometer messages.
        std::string frame_id_;
        std::string link_name_;

        /// \brief    Topic name for anemometer messages.
        std::string wind_server_reglink_topic_;
        std::string wind_server_link_wind_topic_;
        std::string anemometer_topic_;

        /// \brief    Pointer to the world.
        physics::WorldPtr world_;

        /// \brief    Pointer to the model.
        physics::ModelPtr model_;

        /// \brief    Pointer to the link.
        physics::LinkPtr link_;

        /// \brief    Pointer to the update event connection.
        event::ConnectionPtr updateConnection_;

        /// \brief    Anemometer measurement variance m/s.
        double anemometer_var_;
        ignition::math::Vector3d noise_var_;

        /// \brief    Normal distribution for pressure noise.
        std::normal_distribution<double> anemometer_velocity_distribution_X_;
        std::normal_distribution<double> anemometer_velocity_distribution_Y_;
        std::normal_distribution<double> anemometer_velocity_distribution_Z_;

        /// \brief    Anemometer sensor message.
        /// \details  This is modified every time OnUpdate() is called,
        //            and then published onto a topic
        physics_msgs::msgs::Wind wind_speed_msg_;
        gz_sensor_msgs::msgs::Anemometer anemometer_message_;
        ignition::math::Vector3d wind_speed_W_;

        std::mt19937 random_generator_X_;
        std::mt19937 random_generator_Y_;
        std::mt19937 random_generator_Z_;
    };
}

#endif // GAZEBO_ANEMOMETER_PLUGIN_H