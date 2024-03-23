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

/* 
 * File:   gazebo_wind3d_model_plugin.h
 * Author: arwillis
 *
 * Created on January 27, 2024, 7:54 AM
 */

#ifndef GAZEBO_DYNAMIC_WIND_PLUGIN_H
#define GAZEBO_DYNAMIC_WIND_PLUGIN_H

#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <mav_msgs/default_topics.h>  // This comes from the mav_comm repo

#include "ConnectGazeboToRosTopic.pb.h"
#include "Wind.pb.h"                  // Wind message
#include "WindServerRegistration.pb.h"

#include "common.h"

namespace gazebo {
    // Default values
    static const bool kPrintOnUpdates = false;
    static const bool kPrintOnPluginLoad = false;
    static const bool kPrintOnMsgCallback = false;
    static const std::string kDefaultWindTopic = "world_wind";
    static const std::string kDefaultWindServerRegisterTopic = "/gazebo/default/wind3d_register_link";
    static const std::string kDefaultWindSpeedTopic = mav_msgs::default_topics::WIND_SPEED;

    static const std::string kDefaultFrameId = "world";
    static const std::string kDefaultNamespace = "";
    static const std::string kDefaultLinkName = "base_link";

    static constexpr double kDefaultWindForceMean = 0.0;
    static constexpr double kDefaultWindForceVariance = 0.0;
    static constexpr double kDefaultWindGustForceMean = 0.0;
    static constexpr double kDefaultWindGustForceVariance = 0.0;

    static constexpr double kDefaultWindGustStart = 10.0;
    static constexpr double kDefaultWindGustDuration = 0.0;

    static constexpr double kDefaultWindSpeedMean = 0.0;
    static constexpr double kDefaultWindSpeedVariance = 0.0;

    static const ignition::math::Vector3d kDefaultWindDirection = ignition::math::Vector3d(1, 0, 0);
    static const ignition::math::Vector3d kDefaultWindGustDirection = ignition::math::Vector3d(0, 1, 0);

    static const std::string kConnectGazeboToRosSubtopic = "connect_gazebo_to_ros_subtopic";
    static const std::string kConnectRosToGazeboSubtopic = "connect_ros_to_gazebo_subtopic";

    /// \brief    This gazebo plugin simulates wind acting on a model.
    /// \details  This plugin publishes on a Gazebo topic and instructs the ROS interface plugin to
    ///           forward the message onto ROS.
    typedef const boost::shared_ptr<const physics_msgs::msgs::Wind>& WindPtr;

    class GazeboDynamicWindPlugin : public ModelPlugin {
    public:

        GazeboDynamicWindPlugin()
        : ModelPlugin(),
        namespace_(kDefaultNamespace),
        wind_server_reglink_topic_(kDefaultWindServerRegisterTopic),
        wind_server_link_wind_topic_(kDefaultNamespace+"/"+kDefaultLinkName+"/"+kDefaultWindSpeedTopic),
        wind_force_mean_(kDefaultWindForceMean),
        wind_force_variance_(kDefaultWindForceVariance),
        frame_id_(kDefaultFrameId),
        link_name_(kDefaultLinkName),
        node_handle_(nullptr),
        pubs_and_subs_created_(false) {
        }

        virtual ~GazeboDynamicWindPlugin();

        virtual void Publish();

    protected:

        /// \brief Load the plugin.
        /// \param[in] _model Pointer to the model that loaded this plugin.
        /// \param[in] _sdf SDF element that describes the plugin.
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        /// \brief Called when the world is updated.
        /// \param[in] _info Update timing information.
        void OnUpdate(const common::UpdateInfo& /*_info*/);
        
        void UpdateForcesAndMoments();

    private:

        /// \brief    Flag that is set to true once CreatePubsAndSubs() is called, used
        ///           to prevent CreatePubsAndSubs() from be called on every OnUpdate().
        bool pubs_and_subs_created_;

        /// \brief    Creates all required publishers and subscribers, incl. routing of messages to/from ROS if required.
        /// \details  Call this once the first time OnUpdate() is called (can't
        ///           be called from Load() because there is no guarantee GazeboRosInterfacePlugin has
        ///           has loaded and listening to ConnectGazeboToRosTopic and ConnectRosToGazeboTopic messages).
        void CreatePubsAndSubs();

        void WindVelocityCallback(WindPtr& wind_speed_msg);
        /// \brief    Pointer to the update event connection.
        event::ConnectionPtr update_connection_;

        physics::WorldPtr world_;
        physics::ModelPtr model_;
        physics::LinkPtr link_;

        std::string namespace_;

        /// \brief    Frame ID for wind messages.
        std::string frame_id_;
        std::string link_name_;
        
        std::string wind_server_reglink_topic_;
        std::string wind_server_link_wind_topic_;
        gazebo::transport::PublisherPtr wind_server_register_pub_;
        gazebo::transport::SubscriberPtr wind_server_link_wind_msg_sub_;

        double wind_force_mean_;
        double wind_force_variance_;

        gazebo::transport::NodePtr node_handle_;

        /// \brief    Gazebo message for sending wind data.
        /// \details  This is defined at the class scope so that it is re-created
        ///           every time a wind message needs to be sent, increasing performance.
        //gz_geometry_msgs::WrenchStamped wrench_stamped_msg_;

        /// \brief    Gazebo message for sending wind speed data.
        /// \details  This is defined at the class scope so that it is re-created
        ///           every time a wind speed message needs to be sent, increasing performance.
        physics_msgs::msgs::Wind wind_speed_msg_;
        ignition::math::Vector3d wind_speed_W_;
    };
}

#endif /* GAZEBO_DYNAMIC_WIND_PLUGIN_H */

