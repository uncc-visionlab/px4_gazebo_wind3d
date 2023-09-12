/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Anton Matosov
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


#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H

#include <string>
#include <random>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>

#include "Wind.pb.h"
//#include "WindSpeed.pb.h"             // Wind speed message
//#include "WrenchStamped.pb.h"         // Wind force message
#include <nanoflann/nanoflann.hpp>
#include <nanoflann/utils.h>

typedef float num_t;

namespace gazebo {
    // Default values
    static const std::string kDefaultNamespace = "";
    static const std::string kDefaultFrameId = "world";

    static constexpr double kDefaultWindVelocityMean = 0.0;
    static constexpr double kDefaultWindVelocityMax = 100.0;
    static constexpr double kDefaultWindVelocityVariance = 0.0;
    static constexpr double kDefaultWindGustVelocityMean = 0.0;
    static constexpr double kDefaultWindGustVelocityMax = 10.0;
    static constexpr double kDefaultWindGustVelocityVariance = 0.0;

    static constexpr double kDefaultWindGustStart = 10.0;
    static constexpr double kDefaultWindGustDuration = 0.0;

    static const ignition::math::Vector3d kDefaultWindDirectionMean = ignition::math::Vector3d(1, 0, 0);
    static const ignition::math::Vector3d kDefaultWindGustDirectionMean = ignition::math::Vector3d(0, 1, 0);
    static constexpr double kDefaultWindDirectionVariance = 0.0;
    static constexpr double kDefaultWindGustDirectionVariance = 0.0;

/// \brief This gazebo plugin simulates wind acting on a model.
    class GazeboWindPlugin : public WorldPlugin {
    public:
        GazeboWindPlugin()
                : WorldPlugin(),
                  namespace_(kDefaultNamespace),
                  wind_pub_topic_("world_wind"),
                  wind_velocity_mean_(kDefaultWindVelocityMean),
                  wind_velocity_max_(kDefaultWindVelocityMax),
                  wind_velocity_variance_(kDefaultWindVelocityVariance),
                  wind_gust_velocity_mean_(kDefaultWindGustVelocityMean),
                  wind_gust_velocity_max_(kDefaultWindGustVelocityMax),
                  wind_gust_velocity_variance_(kDefaultWindGustVelocityVariance),
                  wind_direction_mean_(kDefaultWindDirectionMean),
                  wind_direction_variance_(kDefaultWindDirectionVariance),
                  wind_gust_direction_mean_(kDefaultWindGustDirectionMean),
                  wind_gust_direction_variance_(kDefaultWindGustDirectionVariance),
                  frame_id_(kDefaultFrameId),
                  pub_interval_(0.5),
                  node_handle_(NULL) {}

        virtual ~GazeboWindPlugin();

    protected:
        /// \brief Load the plugin.
        /// \param[in] _model Pointer to the model that loaded this plugin.
        /// \param[in] _sdf SDF element that describes the plugin.
        void Load(physics::WorldPtr world, sdf::ElementPtr sdf);

        /// \brief Called when the world is updated.
        /// \param[in] _info Update timing information.
        void OnUpdate(const common::UpdateInfo & /*_info*/);

    private:
        /// \brief    Flag that is set to true once CreatePubsAndSubs() is called, used
        ///           to prevent CreatePubsAndSubs() from be called on every OnUpdate().
        bool pubs_and_subs_created_;

        /// \brief    Creates all required publishers and subscribers, incl. routing of messages to/from ROS if required.
        /// \details  Call this once the first time OnUpdate() is called (can't
        ///           be called from Load() because there is no guarantee GazeboRosInterfacePlugin has
        ///           has loaded and listening to ConnectGazeboToRosTopic and ConnectRosToGazeboTopic messages).
        void CreatePubsAndSubs();

        /// \brief Pointer to the update event connection.
        event::ConnectionPtr update_connection_;

        physics::WorldPtr world_;

        std::string namespace_;

        std::string frame_id_;
        std::string wind_pub_topic_;

        double wind_velocity_mean_;
        double wind_velocity_max_;
        double wind_velocity_variance_;
        double wind_gust_velocity_mean_;
        double wind_gust_velocity_max_;
        double wind_gust_velocity_variance_;
        double pub_interval_;
        std::default_random_engine wind_velocity_generator_;
        std::normal_distribution<double> wind_velocity_distribution_;
        std::default_random_engine wind_gust_velocity_generator_;
        std::normal_distribution<double> wind_gust_velocity_distribution_;

        ignition::math::Vector3d wind_direction_mean_;
        ignition::math::Vector3d wind_gust_direction_mean_;
        double wind_direction_variance_;
        double wind_gust_direction_variance_;
        std::default_random_engine wind_direction_generator_;
        std::normal_distribution<double> wind_direction_distribution_X_;
        std::normal_distribution<double> wind_direction_distribution_Y_;
        std::normal_distribution<double> wind_direction_distribution_Z_;
        std::default_random_engine wind_gust_direction_generator_;
        std::normal_distribution<double> wind_gust_direction_distribution_X_;
        std::normal_distribution<double> wind_gust_direction_distribution_Y_;
        std::normal_distribution<double> wind_gust_direction_distribution_Z_;

        common::Time wind_gust_end_;
        common::Time wind_gust_start_;
        common::Time last_time_;

        /// \brief    Variables for custom wind field generation.
        bool use_custom_static_wind_field_;
        std::string link_name_;
        std::vector<physics::ModelPtr> model_;
        physics::LinkPtr link_;
        using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
                nanoflann::L2_Simple_Adaptor<num_t, SampledVectorField<num_t, 3>>,
                SampledVectorField<num_t, 3>, 3 /* dim */>;
        SampledVectorField<num_t,3> pt_cloud_vec3;
        my_kd_tree_t *windfield_kdtree;

        float min_x_;
        float min_y_;
        int n_x_;
        int n_y_;
        float res_x_;
        float res_y_;
        std::vector<float> vertical_spacing_factors_;
        std::vector<float> bottom_z_;
        std::vector<float> top_z_;
        std::vector<float> u_;
        std::vector<float> v_;
        std::vector<float> w_;


        /// \brief  Reads wind data from a text file and saves it.
        /// \param[in] custom_wind_field_path Path to the wind field from ~/.ros.
        void ReadCustomWindField(std::string &custom_wind_field_path);

        transport::NodePtr node_handle_;
        transport::PublisherPtr wind_pub_;

        physics_msgs::msgs::Wind wind_msg;

        /// \brief    Gazebo message for sending wind data.
        /// \details  This is defined at the class scope so that it is re-created
        ///           everytime a wind message needs to be sent, increasing performance.
//        gz_geometry_msgs::WrenchStamped wrench_stamped_msg_;

        /// \brief    Gazebo message for sending wind speed data.
        /// \details  This is defined at the class scope so that it is re-created
        ///           everytime a wind speed message needs to be sent, increasing performance.
//        gz_mav_msgs::WindSpeed wind_speed_msg_;
    };
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H
