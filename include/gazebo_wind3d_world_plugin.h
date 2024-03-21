/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Anton Matosov
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


#ifndef GAZEBO_WIND3D_WORLD_PLUGIN_H
#define GAZEBO_WIND3D_WORLD_PLUGIN_H

#include <string>
#include <random>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>

#include "Wind.pb.h"
#include "WindServerRegistration.pb.h"
//#include "WindSpeed.pb.h"             // Wind speed message
//#include "WrenchStamped.pb.h"         // Wind force message
#include <nanoflann/nanoflann.hpp>
#include <nanoflann/utils.h>

// Related work in other Github repositories
//
// https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/src/gazebo_airship_dynamics_plugin.cpp
// https://github.com/ethz-asl/rotors_simulator/blob/master/rotors_gazebo/launch/mav_with_wind_gust.launch
// https://github.com/gazebosim/gazebo-classic/blob/gazebo11/plugins/WindPlugin.cc

typedef float num_t;

namespace gazebo {
    // Default values
    static const bool kPrintOnUpdates = false;
    static const std::string kDefaultWindTopic = "world_wind";
    static const std::string kDefaultWindServerRegisterTopic = "/gazebo/default/wind3d_register_link";

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

    typedef const boost::shared_ptr<const wind3d_msgs::msgs::WindServerRegistration>& WindServerRegistrationPtr;
    typedef const boost::shared_ptr<const physics_msgs::msgs::Wind>& GzWindSpeedMsgPtr;

    /// \brief This gazebo plugin simulates wind acting on a model.

    class GazeboWind3DWorldPlugin : public WorldPlugin {
    public:

        GazeboWind3DWorldPlugin()
        : WorldPlugin(),
        wind_server_reglink_topic_(kDefaultWindServerRegisterTopic),
        wind_pub_topic_(kDefaultWindTopic),
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
        pub_interval_(0.5),
        node_handle_(NULL) {

        }

        virtual ~GazeboWind3DWorldPlugin();

        void RegisterLinkCallback(WindServerRegistrationPtr msg);

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

        template <typename T>
        bool readCSV(std::string datafile_path, std::vector<std::vector<T>> &data) {
            std::ifstream fin;
            fin.open(datafile_path);
            if (!fin.is_open()) {
                gzerr << __FUNCTION__ << "Error reading file " << datafile_path << std::endl;                
                return false;
            }
            // Read the data from the file
            std::string line;
            double dvalue;
            int linenum = 0;
            while (getline(fin, line)) {
                linenum++;
                //std::cout << "read line[" << linenum << "]: " << line << std::endl;
                std::vector<double> row;
                std::string value;
                std::stringstream ss(line);
                bool lineOK = true;

                //std::cout << "line[" << linenum << "] = {";
                while (getline(ss, value, ',')) {
                    double dvalue;
                    try {
                        dvalue = std::stod(value);
                    } catch (...) {
//                        gzdbg << "[gazebo_wind3d_world_plugin] Could not convert string to double, skipping value on line " << linenum
//                                << std::endl;
//                        gzdbg << "[gazebo_wind3d_world_plugin] line[" << linenum << "]=" << line << std::endl;
                        lineOK = false;
                        break;
                    }
                    row.push_back(dvalue);
                    //std::cout << dvalue << ", ";
                }
                //std::cout << std::endl;
                if (lineOK) {
                    //std::cout << "(X,Y,Z), (U,V,W) = " <<
                    //          "(" << row[0] << ", " << row[1] << ", " << row[2] << "), " <<
                    //          "(" << row[3] << ", " << row[4] << ", " << row[5] << ")" << std::endl;
                    data.push_back(row);
                }
            }
            fin.close();
            return true;
        }

        /// \brief Pointer to the update event connection.
        event::ConnectionPtr update_connection_;

        physics::WorldPtr world_;

        //        std::string namespace_;

        std::string frame_id_;
        std::string wind_server_reglink_topic_;
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
        bool use_custom_dynamic_wind_field_;
        //        std::string link_name_;
        //        std::vector<physics::ModelPtr> model_;
        //        physics::LinkPtr link_;
        using static_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
                nanoflann::L2_Simple_Adaptor<num_t, SampledVectorField<num_t, 3 >>,
                SampledVectorField<num_t, 3>, 3 /* dim */>;
        SampledVectorField<num_t, 3> pt_cloud_vec3;
        
        using fftfunc_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
                nanoflann::L2_Simple_Adaptor<num_t, SampledFourierRealFunctionField<num_t, 3, 10>>,
                SampledFourierRealFunctionField<num_t, 3, 10>, 3 /* dim */>;
        SampledFourierRealFunctionField<num_t, 3, 10> pt_cloud_fftfunc;
        
        
        static_kd_tree_t *windfield_kdtree;
        fftfunc_kd_tree_t *windfield_fft_kdtree;


        /// \brief  Reads wind data from a text file and saves it.
        /// \param[in] custom_wind_field_path Path to the wind field from ~/.ros.
        void ReadCustomStaticWindField(std::string &custom_wind_field_path);

        void ReadCustomDynamicWindField(std::string & wind_field_datafile_path);

        gazebo::transport::SubscriberPtr wind_register_sub_;

        std::vector<std::string> registered_link_name_list_;
        std::vector<physics::LinkPtr> registered_link_list_;
        std::vector<physics::ModelPtr> registered_model_list_;
        std::vector<std::string> registered_namespace_list_;
        std::vector<std::string> registered_wind_server_link_wind_topic_list_;
        std::vector<transport::PublisherPtr> registered_link_wind_publisher_list_;

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

#endif // GAZEBO_WIND3D_WORLD_PLUGIN_H
