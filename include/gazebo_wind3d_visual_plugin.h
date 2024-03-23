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

#ifndef GAZEBO_WIND3D_VISUAL_PLUGIN_H
#define GAZEBO_WIND3D_VISUAL_PLUGIN_H

#include <random>

#include <glog/logging.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"

#include "Wind.pb.h"

#include <mav_msgs/default_topics.h>  // This comes from the mav_comm repo

#include "common.h"

namespace gazebo {
    namespace rendering {
        // Constants
        static const bool kPrintOnPluginLoad = true;
        static const bool kPrintOnUpdates = false;
        static const bool kPrintOnMsgCallback = true;
        static const std::string kDefaultNamespace = "";
        static const std::string kDefaultWindServerRegisterTopic = "/gazebo/default/wind3d_register_link";
        static const std::string kDefaultWindServerLinkTopic = "/gazebo/default/wind3d_visual";
        static const std::string kDefaultWindSpeedTopic = mav_msgs::default_topics::WIND_SPEED;
        static const std::string kConnectGazeboToRosSubtopic = "connect_gazebo_to_ros_subtopic";
        static const std::string kConnectRosToGazeboSubtopic = "connect_ros_to_gazebo_subtopic";

        typedef const boost::shared_ptr<const physics_msgs::msgs::Wind>& WindPtr;

        class GazeboWind3DVisualPlugin : public VisualPlugin {
        public:
            /// \brief    Constructor.

            GazeboWind3DVisualPlugin()
            : VisualPlugin(),
            namespace_(kDefaultNamespace),
            wind_server_reglink_topic_(kDefaultWindServerRegisterTopic),
            wind_server_link_wind_topic_(kDefaultWindServerLinkTopic),
            node_handle_(nullptr),
            pubs_and_subs_created_(false) {
            }

            /// \brief    Destructor.
            virtual ~GazeboWind3DVisualPlugin();

        protected:
            /// \brief    Called when the plugin is first created, and after the world
            ///           has been loaded. This function should not be blocking.
            void Load(VisualPtr _parent, sdf::ElementPtr _sdf);

            /// \brief  	This gets called by the world update start event.
            void OnUpdate();

        private:
            /// \brief    Flag that is set to true once CreatePubsAndSubs() is called, used
            ///           to prevent CreatePubsAndSubs() from be called on every OnUpdate().
            bool pubs_and_subs_created_;

            /// \brief    Creates all required publishers and subscribers, incl. routing of messages to/from ROS if required.
            /// \details  Call this once the first time OnUpdate() is called (can't
            ///           be called from Load() because there is no guarantee GazeboRosInterfacePlugin has
            ///           has loaded and listening to ConnectGazeboToRosTopic and ConnectRosToGazeboTopic messages).
            void CreatePubsAndSubs();

            void WindVelocity(WindPtr& wind_speed_msg);

            /// \brief    Handle for the Gazebo node.
            gazebo::transport::NodePtr node_handle_;

            /// \brief    Topic name for anemometer messages.
            std::string wind_server_reglink_topic_;
            std::string wind_server_link_wind_topic_;

            /// \brief    Wind 3D plugin publishers and subscribers
            gazebo::transport::PublisherPtr wind_server_register_pub_;
            gazebo::transport::SubscriberPtr wind_server_link_wind_msg_sub_;

            std::string link_name_;
            std::string model_name_;
            /// \brief    Transport namespace.
            std::string namespace_;

            /// \brief The visual pointer used to visualize the force.
            VisualPtr visual_;

            /// \brief The scene pointer.
            ScenePtr scene_;

            gazebo::rendering::ArrowVisualPtr arrow_ptr_;
            ignition::math::Vector3d wind_velocity_;
            /// \brief    Pointer to the world.
            physics::WorldPtr world_;

            /// \brief    Pointer to the update event connection.
            event::ConnectionPtr updateConnection_;
        };
    }
}

#endif // GAZEBO_WIND3D_VISUAL_PLUGIN_H