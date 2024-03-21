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

#include "gazebo_wind3d_world_plugin.h"
#include "common.h"

namespace gazebo {

    GazeboWind3DWorldPlugin::~GazeboWind3DWorldPlugin() {
        update_connection_->~Connection();
        if (windfield_kdtree) {
            delete windfield_kdtree;
        }
    }

    void GazeboWind3DWorldPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf) {
        world_ = world;
        double wind_gust_start = kDefaultWindGustStart;
        double wind_gust_duration = kDefaultWindGustDuration;

        node_handle_ = transport::NodePtr(new transport::Node());
        node_handle_->Init();
        //node_handle_->Init(namespace_);

        getSdfParam<std::string>(sdf, "windPubTopic", wind_pub_topic_, wind_pub_topic_);
        getSdfParam<std::string>(sdf, "windServerRegisterLinkTopic", wind_server_reglink_topic_, wind_server_reglink_topic_);

        // Wind topic publishing rates
        double pub_rate = 2.0;
        getSdfParam<double>(sdf, "publishRate", pub_rate, pub_rate);
        pub_interval_ = (pub_rate > 0.0) ? 1 / pub_rate : 0.0;

        // Get the wind params from SDF.
        // Check if a custom static wind field should be used.
        getSdfParam<bool>(sdf, "useCustomStaticWindField", use_custom_static_wind_field_, false);
        getSdfParam<bool>(sdf, "useCustomDynamicWindField", use_custom_dynamic_wind_field_, false);

        if (use_custom_static_wind_field_) {
            gzdbg << "[gazebo_wind3d_world_plugin] Using custom spatially varying wind field from text file.\n";
            // Get the wind field text file path, read it and save data.
            std::string custom_static_wind_field_path;
            getSdfParam<std::string>(sdf, "customStaticWindFieldPath", custom_static_wind_field_path,
                    custom_static_wind_field_path);

            ReadCustomStaticWindField(custom_static_wind_field_path);
            //const std::vector<gazebo::physics::ModelPtr> models = world_->Models();
            //for (auto model_ : models) {
            //    gzdbg << "[gazebo_wind3d_world_plugin] Inspecting model: " << model_->GetName() << "." << std::endl;
            //    link_ = model_->GetLink(link_name_);
            //    if (link_ != NULL) {
            //        gzdbg << "[gazebo_wind3d_world_plugin] " <<
            //                model_->GetName() << " has link " << link_->GetName() << "." << std::endl;
            //    } else {
            //        gzdbg << "[gazebo_wind3d_world_plugin] Model: " << model_->GetName() <<
            //                " does not have link " << link_name_ << "." << std::endl;
            //        //gzthrow("[gazebo_wind_plugin] Couldn't find specified link \"" << link_name_ << "\".");
            //    }
            //}
        } else if (use_custom_dynamic_wind_field_) {
            gzdbg << "[gazebo_wind3d_world_plugin] Using custom spatially and time varying wind field from text files\n";
            // Get the wind field text file path, read it and save data.
            std::string custom_dyn_wind_field_path;
            getSdfParam<std::string>(sdf, "customDynamicWindFieldPath", custom_dyn_wind_field_path,
                    custom_dyn_wind_field_path);

            ReadCustomDynamicWindField(custom_dyn_wind_field_path);
        } else {
            gzdbg << "[gazebo_wind3d_world_plugin] Using user-defined constant wind field and gusts.\n";

            // Get the wind params from SDF.
            getSdfParam<double>(sdf, "windVelocityMean", wind_velocity_mean_, wind_velocity_mean_);
            getSdfParam<double>(sdf, "windVelocityMax", wind_velocity_max_, wind_velocity_max_);
            getSdfParam<double>(sdf, "windVelocityVariance", wind_velocity_variance_, wind_velocity_variance_);
            getSdfParam<ignition::math::Vector3d>(sdf, "windDirectionMean", wind_direction_mean_, wind_direction_mean_);
            getSdfParam<double>(sdf, "windDirectionVariance", wind_direction_variance_, wind_direction_variance_);

            // Get the wind gust params from SDF.
            getSdfParam<double>(sdf, "windGustStart", wind_gust_start, wind_gust_start);
            getSdfParam<double>(sdf, "windGustDuration", wind_gust_duration, wind_gust_duration);
            getSdfParam<double>(sdf, "windGustVelocityMean", wind_gust_velocity_mean_, wind_gust_velocity_mean_);
            getSdfParam<double>(sdf, "windGustVelocityMax", wind_gust_velocity_max_, wind_gust_velocity_max_);
            getSdfParam<double>(sdf, "windGustVelocityVariance", wind_gust_velocity_variance_,
                    wind_gust_velocity_variance_);
            getSdfParam<ignition::math::Vector3d>(sdf, "windGustDirectionMean", wind_gust_direction_mean_,
                    wind_gust_direction_mean_);
            getSdfParam<double>(sdf, "windGustDirectionVariance", wind_gust_direction_variance_,
                    wind_gust_direction_variance_);

            wind_direction_mean_.Normalize();
            wind_gust_direction_mean_.Normalize();
            wind_gust_start_ = common::Time(wind_gust_start);
            wind_gust_end_ = common::Time(wind_gust_start + wind_gust_duration);
            // Set random wind velocity mean and standard deviation
            wind_velocity_distribution_.param(
                    std::normal_distribution<double>::param_type(wind_velocity_mean_,
                    sqrt(wind_velocity_variance_)));
            // Set random wind direction mean and standard deviation
            wind_direction_distribution_X_.param(
                    std::normal_distribution<double>::param_type(wind_direction_mean_.X(),
                    sqrt(wind_direction_variance_)));
            wind_direction_distribution_Y_.param(
                    std::normal_distribution<double>::param_type(wind_direction_mean_.Y(),
                    sqrt(wind_direction_variance_)));
            wind_direction_distribution_Z_.param(
                    std::normal_distribution<double>::param_type(wind_direction_mean_.Z(),
                    sqrt(wind_direction_variance_)));
            // Set random wind gust velocity mean and standard deviation
            wind_gust_velocity_distribution_.param(
                    std::normal_distribution<double>::param_type(wind_gust_velocity_mean_,
                    sqrt(wind_gust_velocity_variance_)));
            // Set random wind gust direction mean and standard deviation
            wind_gust_direction_distribution_X_.param(
                    std::normal_distribution<double>::param_type(wind_gust_direction_mean_.X(),
                    sqrt(wind_gust_direction_variance_)));
            wind_gust_direction_distribution_Y_.param(
                    std::normal_distribution<double>::param_type(wind_gust_direction_mean_.Y(),
                    sqrt(wind_gust_direction_variance_)));
            wind_gust_direction_distribution_Z_.param(
                    std::normal_distribution<double>::param_type(wind_gust_direction_mean_.Z(),
                    sqrt(wind_gust_direction_variance_)));
        }

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        update_connection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GazeboWind3DWorldPlugin::OnUpdate, this, _1));

#if GAZEBO_MAJOR_VERSION >= 9
        last_time_ = world_->SimTime();
#else
        last_time_ = world_->GetSimTime();
#endif
    }

    void GazeboWind3DWorldPlugin::RegisterLinkCallback(WindServerRegistrationPtr msg) {
        //gzdbg << __FUNCTION__ << "() called." << std::endl;
        const std::string& model_name = msg->model_name();
        const std::string& link_name = msg->link_name();
        const std::string& namespace_val = msg->namespace_();
        const std::string& link_wind_topic = msg->link_wind_topic();
        physics::LinkPtr currentModelLinkPtr = NULL;
        gazebo::physics::ModelPtr currentModelPtr = NULL;
        //const std::vector<gazebo::physics::ModelPtr> models = world_->Models();
        //for (gazebo::physics::ModelPtr model_ : models) {
        //    currentModelLinkPtr = model_->GetLink(link_name);
        //    currentModelPtr = model_;
        //    currentModelPtr = world_->ModelByName(model_name);
        //    gzdbg << "[px4_gazebo_wind3d] Inspecting model: " << model_->GetName() << "." << std::endl;
        //    if (currentModelLinkPtr != NULL) {
        //        gzdbg << "[gazebo_wind3d_world_plugin] " <<
        //                model_->GetName() << " has link " << currentModelLinkPtr->GetName() << "." << std::endl;
        //        break;
        //    } else {
        //        gzdbg << "[gazebo_wind3d_world_plugin] Model: " << model_->GetName() <<
        //                "does not have link " << link_name << "." << std::endl;
        //        //gzthrow("[gazebo_wind_plugin] Couldn't find specified link \"" << link_name_ << "\".");
        //    }
        //}
        currentModelPtr = world_->ModelByName(model_name);
        if (currentModelPtr == NULL) {
            gzdbg << "[gazebo_wind3d_world_plugin] Model: " << model_name << "does not exist ." << std::endl;
            return;
        }
        currentModelLinkPtr = currentModelPtr->GetLink(link_name);

        if (currentModelLinkPtr != NULL) {
            registered_link_name_list_.push_back(link_name);
            registered_link_list_.push_back(currentModelLinkPtr);
            registered_namespace_list_.push_back(namespace_val);
            registered_wind_server_link_wind_topic_list_.push_back(link_wind_topic);
            //transport::PublisherPtr link_wind_pub_ = node_handle_->Advertise<physics_msgs::msgs::Wind>(msg->link_wind_topic(), 10);
            registered_link_wind_publisher_list_.push_back(
                    node_handle_->Advertise<physics_msgs::msgs::Wind>(msg->link_wind_topic(), 10));
            gzdbg << __FUNCTION__ << "() Registered (Model, Namespace, Link, Topic) = (" << currentModelPtr->GetName()
                    << ", " << namespace_val << ", " << link_name << ", " << link_wind_topic << ") to the world wind server." << std::endl;
        } else {
            gzdbg << __FUNCTION__ << "() Model: " << currentModelPtr->GetName() <<
                    "does not have link " << link_name << "." << std::endl;
        }
    }

    // This gets called by the world update start event.

    void GazeboWind3DWorldPlugin::OnUpdate(const common::UpdateInfo & _info) {
        if (kPrintOnUpdates) {
            gzdbg << __FUNCTION__ << "() called." << std::endl;
        }
        // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
        common::Time now = world_->SimTime();
#else
        common::Time now = world_->GetSimTime();
#endif
        if (!pubs_and_subs_created_) {
            CreatePubsAndSubs();
            pubs_and_subs_created_ = true;
        }

        if ((now - last_time_).Double() < pub_interval_ || pub_interval_ == 0.0) {
            return;
        }
        last_time_ = now;

        // Calculate the wind force.
        // Get normal distribution wind strength
        if (!use_custom_static_wind_field_) {
            gazebo::msgs::Vector3d* wind_v_ptr = new gazebo::msgs::Vector3d();
            double wind_strength = std::abs(wind_velocity_distribution_(wind_velocity_generator_));
            wind_strength = (wind_strength > wind_velocity_max_) ? wind_velocity_max_ : wind_strength;
            // Get normal distribution wind direction
            ignition::math::Vector3d wind_direction;
            wind_direction.X() = wind_direction_distribution_X_(wind_direction_generator_);
            wind_direction.Y() = wind_direction_distribution_Y_(wind_direction_generator_);
            wind_direction.Z() = wind_direction_distribution_Z_(wind_direction_generator_);
            // Calculate total wind velocity
            ignition::math::Vector3d wind = wind_strength * wind_direction;

            ignition::math::Vector3d wind_gust(0, 0, 0);
            // Calculate the wind gust velocity.
            if (now >= wind_gust_start_ && now < wind_gust_end_) {
                // Get normal distribution wind gust strength
                double wind_gust_strength = std::abs(wind_gust_velocity_distribution_(wind_gust_velocity_generator_));
                wind_gust_strength = (wind_gust_strength > wind_gust_velocity_max_) ? wind_gust_velocity_max_
                        : wind_gust_strength;
                // Get normal distribution wind gust direction
                ignition::math::Vector3d wind_gust_direction;
                wind_gust_direction.X() = wind_gust_direction_distribution_X_(wind_gust_direction_generator_);
                wind_gust_direction.Y() = wind_gust_direction_distribution_Y_(wind_gust_direction_generator_);
                wind_gust_direction.Z() = wind_gust_direction_distribution_Z_(wind_gust_direction_generator_);
                wind_gust = wind_gust_strength * wind_gust_direction;
            }

            wind_v_ptr->set_x(wind.X() + wind_gust.X());
            wind_v_ptr->set_y(wind.Y() + wind_gust.Y());
            wind_v_ptr->set_z(wind.Z() + wind_gust.Z());
            wind_msg.set_frame_id(frame_id_);
            wind_msg.set_time_usec(now.Double() * 1e6);
            wind_msg.set_allocated_velocity(wind_v_ptr);
            wind_pub_->Publish(wind_msg);
        } else {
            if (registered_link_list_.size() == 0) {
                return;
            }
            //if (link_ == NULL) {
            //    std::cout << "[gazebo_wind3d_world_plugin] Error link " <<
            //            link_name_ << " could not be found." << std::endl;
            //    return;
            //}            
            int index = 0;
            for (physics::LinkPtr link : registered_link_list_) {
                ignition::math::Vector3d link_position = link->WorldPose().Pos();
                const num_t query_pt[3] = {(float) link_position.X(), (float) link_position.Y(), (float) link_position.Z()};
                // ----------------------------------------------------------------
                // knnSearch():  Perform a search for the N closest points
                // ----------------------------------------------------------------
                size_t num_results = 5;
                std::vector<uint32_t> ret_index(num_results);
                std::vector<num_t> out_dist_sqr(num_results);

                num_results = windfield_kdtree->knnSearch(&query_pt[0], num_results, &ret_index[0],
                        &out_dist_sqr[0]);

                // In case of less points in the tree than requested:
                ret_index.resize(num_results);
                out_dist_sqr.resize(num_results);

                //std::cout << "knnSearch(): num_results=" << num_results << std::endl;
                int pt_idx = 0;
                ignition::math::Vector3d wind_direction(0, 0, 0);
                float total_invdistance_sqr = 0;
                for (size_t i = 0; i < num_results; i++) {
                    total_invdistance_sqr += 1.0f / out_dist_sqr[i];
                }
                for (size_t i = 0; i < num_results; i++) {
                    // std::cout << "idx[" << i << "]=" << ret_index[i] <<
                    //        " (X,Y,Z) = (" << pt_cloud_vec3.pts[ret_index[i]].x << ", " <<
                    //        pt_cloud_vec3.pts[ret_index[i]].y << ", " <<
                    //        pt_cloud_vec3.pts[ret_index[i]].z << ") " << " dist[" << i
                    //        << "]=" << out_dist_sqr[i] << std::endl;
                    pt_idx = ret_index[i];
                    wind_direction.X() +=
                            pt_cloud_vec3._data[pt_idx][0] * (1.0f / out_dist_sqr[i]) / total_invdistance_sqr;
                    wind_direction.Y() +=
                            pt_cloud_vec3._data[pt_idx][1] * (1.0f / out_dist_sqr[i]) / total_invdistance_sqr;
                    wind_direction.Z() +=
                            pt_cloud_vec3._data[pt_idx][2] * (1.0f / out_dist_sqr[i]) / total_invdistance_sqr;
                    pt_cloud_vec3._data[pt_idx][0];
                }

                double wind_strength = wind_direction.Length();
                if (wind_strength > wind_velocity_max_) {
                    wind_direction *= wind_velocity_max_ / wind_strength;
                }
                // Get normal distribution wind direction
                ignition::math::Vector3d wind = wind_direction;
                gazebo::msgs::Vector3d* wind_v_ptr = new gazebo::msgs::Vector3d();
                wind_v_ptr->set_x(wind.X());
                wind_v_ptr->set_y(wind.Y());
                wind_v_ptr->set_z(wind.Z());
                wind_msg.set_frame_id(frame_id_);
                wind_msg.set_time_usec(now.Double() * 1e6);
                wind_msg.set_allocated_velocity(wind_v_ptr);
                //wind_pub_->Publish(wind_msg);
                registered_link_wind_publisher_list_[index++]->Publish(wind_msg);
            }
        }
    }

    void GazeboWind3DWorldPlugin::CreatePubsAndSubs() {
        wind_pub_ = node_handle_->Advertise<physics_msgs::msgs::Wind>("~/" + wind_pub_topic_, 10);
        wind_register_sub_ = node_handle_->Subscribe<wind3d_msgs::msgs::WindServerRegistration>(wind_server_reglink_topic_,
                &GazeboWind3DWorldPlugin::RegisterLinkCallback, this);
        // Create temporary "ConnectGazeboToRosTopic" publisher and message.
        //        gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
        //                node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
        //                        "~/" + kConnectGazeboToRosSubtopic, 1);
        //
        //        gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;

        // ============================================ //
        // ========= WRENCH STAMPED MSG SETUP ========= //
        // ============================================ //
        //        wind_force_pub_ = node_handle_->Advertise<gz_geometry_msgs::WrenchStamped>(
        //                "~/" + namespace_ + "/" + wind_force_pub_topic_, 1);
        //        wind_pub_ = node_handle_->Advertise<physics_msgs::msgs::Wind>("~/" + wind_pub_topic_, 10);

        // connect_gazebo_to_ros_topic_msg.set_gazebo_namespace(namespace_);
        //        connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
        //                                                         wind_force_pub_topic_);
        //        connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
        //                                                      wind_force_pub_topic_);
        //        connect_gazebo_to_ros_topic_msg.set_msgtype(
        //                gz_std_msgs::ConnectGazeboToRosTopic::WRENCH_STAMPED);
        //        connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
        //                                                 true);

        // ============================================ //
        // ========== WIND SPEED MSG SETUP ============ //
        // ============================================ //
        //        wind_speed_pub_ = node_handle_->Advertise<gz_mav_msgs::WindSpeed>(
        //                "~/" + namespace_ + "/" + wind_speed_pub_topic_, 1);
        //
        //        connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
        //                                                         wind_speed_pub_topic_);
        //        connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
        //                                                      wind_speed_pub_topic_);
        //        connect_gazebo_to_ros_topic_msg.set_msgtype(
        //                gz_std_msgs::ConnectGazeboToRosTopic::WIND_SPEED);
        //        connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
        //                                                 true);
    }

    void GazeboWind3DWorldPlugin::ReadCustomStaticWindField(std::string & custom_wind_field_xyz_uvw_path) {
        // Create a vector to store the data
        std::vector<std::vector<double>> data;
        bool csvReadOK = readCSV(custom_wind_field_xyz_uvw_path, data);
        if (!csvReadOK) {
            gzerr << __FUNCTION__ << "[gazebo_wind3d_world_plugin] Could not open custom wind field CSV file." << std::endl;
            return;
        }
        int num_points = data.size();
        pt_cloud_vec3.pts.resize(num_points);
        pt_cloud_vec3._data.resize(num_points);
        int index = 0;
        float pt_avg[3]{0.f, 0.f, 0.f};
        gzdbg << __FUNCTION__ << " " << num_points << " rows loaded from custom XYZ varying wind CSV data file." << std::endl;
        for (std::vector<double> row : data) {
            // Generating point cloud of wind field vector measurements            
            pt_cloud_vec3.pts[index].x = row[0];
            pt_cloud_vec3.pts[index].y = row[1];
            pt_cloud_vec3.pts[index].z = row[2];
            for (int j = 0; j < 3; j++)
                pt_avg[j] += row[j] / num_points;
            pt_cloud_vec3._data[index][0] = row[3];
            pt_cloud_vec3._data[index][1] = row[4];
            pt_cloud_vec3._data[index][2] = row[5];
            // Print the data
            //std::cout << "(X,Y,Z), (U,V,W) = (" << pt_cloud_vec3.pts[i].x << ", " <<
            //        pt_cloud_vec3.pts[i].y << ", " << pt_cloud_vec3.pts[i].z << "), " <<
            //        "(" << pt_cloud_vec3._data[0][0] << ", " << pt_cloud_vec3._data[0][1] << ", " <<
            //        pt_cloud_vec3._data[0][3] << ")" << std::endl;
            index++;
        }
        gzdbg << "[gazebo_wind3d_world_plugin] Average sample position is (x,y,z)=(" <<
                pt_avg[0] << ", " << pt_avg[1] << pt_avg[2] << ")." << std::endl;
        //dump_mem_usage();
        windfield_kdtree = new static_kd_tree_t(3, pt_cloud_vec3,{2 /* max elements in a leaf */});
        const num_t query_pt[3] = {pt_avg[0], pt_avg[1], pt_avg[2]};
        // ----------------------------------------------------------------
        // knnSearch():  Perform a search for the N closest points
        // ----------------------------------------------------------------
        {
            size_t num_results = 5;
            std::vector<uint32_t> ret_index(num_results);
            std::vector<num_t> out_dist_sqr(num_results);

            num_results = windfield_kdtree->knnSearch(&query_pt[0], num_results, &ret_index[0],
                    &out_dist_sqr[0]);

            // In case of less points in the tree than requested:
            ret_index.resize(num_results);
            out_dist_sqr.resize(num_results);

            gzdbg << "[gazebo_wind3d_world_plugin] knnSearch(): @ average position with num_results=" << num_results << std::endl;
            for (size_t i = 0; i < num_results; i++)
                gzdbg << "[gazebo_wind3d_world_plugin] idx[" << i << "]=" << ret_index[i] <<
                    " (X,Y,Z) = (" << pt_cloud_vec3.pts[ret_index[i]].x << ", " <<
                    pt_cloud_vec3.pts[ret_index[i]].y << ", " <<
                    pt_cloud_vec3.pts[ret_index[i]].z << ") " << " dist[" << i
                    << "]=" << out_dist_sqr[i] << std::endl;
        }

        // ----------------------------------------------------------------
        // radiusSearch(): Perform a search for the points within search_radius
        // ----------------------------------------------------------------
        //                {
        //                    const num_t search_radius = static_cast<num_t>(3);
        //                    std::vector<nanoflann::ResultItem<uint32_t, num_t>> ret_matches;
        //
        //                    nanoflann::SearchParameters params;
        //                    params.sorted = false;
        //
        //                    const size_t nMatches = index.radiusSearch(&query_pt[0], search_radius, ret_matches, params);
        //                    std::cout << "radiusSearch(): radius=" << search_radius << " -> " << nMatches
        //                              << " matches" << std::endl;
        //                    for (size_t i = 0; i < nMatches; i++)
        //                        std::cout << "idx[" << i << "]=" << ret_matches[i].first <<
        //                                  " (X,Y,Z) = (" << pt_cloud_vec3.pts[ret_matches[i].first].x << ", " <<
        //                                  pt_cloud_vec3.pts[ret_matches[i].first].y << ", " <<
        //                                  pt_cloud_vec3.pts[ret_matches[i].first].z << ") " << " dist[" << i
        //                                  << "]=" << ret_matches[i].second << std::endl;
        //                    std::cout << std::endl;
        //                }
        gzdbg << "[gazebo_wind3d_world_plugin] Successfully read custom spatially varying wind field from text file.\n";
    }

    void GazeboWind3DWorldPlugin::ReadCustomDynamicWindField(std::string & wind_field_datafile_path) {
        bool csvReadOK;
        std::vector<std::vector<double>> csv_data;
        csvReadOK = readCSV(wind_field_datafile_path, csv_data);
        int num_points = csv_data.size();
        gzdbg << __FUNCTION__ << " " << num_points << " rows loaded from custom space and time varying wind CSV data files." << std::endl;
        if (!csvReadOK) {
            gzerr << __FUNCTION__ << "[gazebo_wind3d_world_plugin] Could not open custom wind field text file" << std::endl;
            return;
        }
        const std::vector<double>& first_row = csv_data[0];
        int num_coeffs = (int) (first_row.size() - 3) / 9;
        gzdbg << __FUNCTION__ << " First row has " << first_row.size() << " elements." << std::endl;
        gzdbg << __FUNCTION__ << " " << num_coeffs << " FFT coefficients for each (x,y,z) location." << std::endl;
        pt_cloud_fftfunc.pts.resize(num_points);
        for (int dimension = 0; dimension < 3; dimension++) {
            pt_cloud_fftfunc._freq.resize(num_coeffs);
            pt_cloud_fftfunc._real.resize(num_coeffs);
            pt_cloud_fftfunc._imag.resize(num_coeffs);
        }
        int pt_index = 0;
        float pt_avg[3]{0.f, 0.f, 0.f};
        for (std::vector<double> row : csv_data) {
            // Generating point cloud of wind field vector measurements            
            pt_cloud_fftfunc.pts[pt_index].x = row[0];
            pt_cloud_fftfunc.pts[pt_index].y = row[1];
            pt_cloud_fftfunc.pts[pt_index].z = row[2];
            for (int j = 0; j < 3; j++)
                pt_avg[j] += row[j] / num_points;
            for (int coeff_index = 0; coeff_index < num_coeffs; coeff_index++) {
                for (int dimension = 0; dimension < 3; dimension++) {
                    pt_cloud_fftfunc._freq[dimension][coeff_index] = row[coeff_index * 3 * 3 + dimension * 3 + 3];
                    pt_cloud_fftfunc._real[dimension][coeff_index] = row[coeff_index * 3 * 3 + dimension * 3 + 3 + 1];
                    pt_cloud_fftfunc._imag[dimension][coeff_index] = row[coeff_index * 3 * 3 + dimension * 3 + 3 + 2];
                }
            }
            // Print the data
            //std::cout << "(X,Y,Z), (U,V,W) = (" << pt_cloud_vec3.pts[i].x << ", " <<
            //        pt_cloud_vec3.pts[i].y << ", " << pt_cloud_vec3.pts[i].z << "), " <<
            //        "(" << pt_cloud_vec3._data[0][0] << ", " << pt_cloud_vec3._data[0][1] << ", " <<
            //        pt_cloud_vec3._data[0][3] << ")" << std::endl;
            pt_index++;
        }
        gzdbg << "[gazebo_wind3d_world_plugin] Average sample position is (x,y,z)=(" <<
                pt_avg[0] << ", " << pt_avg[1] << pt_avg[2] << ")." << std::endl;
        //dump_mem_usage();
        windfield_fft_kdtree = new fftfunc_kd_tree_t(3, pt_cloud_fftfunc,{2 /* max elements in a leaf */});
        const num_t query_pt[3] = {pt_avg[0], pt_avg[1], pt_avg[2]};
        // ----------------------------------------------------------------
        // knnSearch():  Perform a search for the N closest points
        // ----------------------------------------------------------------
        {
            size_t num_results = 5;
            std::vector<uint32_t> ret_index(num_results);
            std::vector<num_t> out_dist_sqr(num_results);

            num_results = windfield_fft_kdtree->knnSearch(&query_pt[0], num_results, &ret_index[0],
                    &out_dist_sqr[0]);

            // In case of less points in the tree than requested:
            ret_index.resize(num_results);
            out_dist_sqr.resize(num_results);

            gzdbg << "[gazebo_wind3d_world_plugin] knnSearch(): @ average position with num_results=" << num_results << std::endl;
            for (size_t i = 0; i < num_results; i++)
                gzdbg << "[gazebo_wind3d_world_plugin] idx[" << i << "]=" << ret_index[i] <<
                    " (X,Y,Z) = (" << pt_cloud_fftfunc.pts[ret_index[i]].x << ", " <<
                    pt_cloud_fftfunc.pts[ret_index[i]].y << ", " <<
                    pt_cloud_fftfunc.pts[ret_index[i]].z << ") " << " dist[" << i
                    << "]=" << out_dist_sqr[i] << std::endl;
        }
        // Create a vector to store the data
        gzdbg << "[gazebo_wind3d_world_plugin] Successfully read custom spatially and temporally varying wind field from text file.\n";
    }

    GZ_REGISTER_WORLD_PLUGIN(GazeboWind3DWorldPlugin);
}
