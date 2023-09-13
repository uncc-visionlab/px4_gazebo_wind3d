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

#include "gazebo_wind3d_plugin.h"
#include "common.h"

namespace gazebo {

    GazeboWindPlugin::~GazeboWindPlugin() {
        update_connection_->~Connection();
        if (windfield_kdtree) {
            delete windfield_kdtree;
        }
    }

    void GazeboWindPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf) {
        world_ = world;
        double wind_gust_start = kDefaultWindGustStart;
        double wind_gust_duration = kDefaultWindGustDuration;

        if (sdf->HasElement("robotNamespace")) {
            namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
        } else {
            gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";
        }

        node_handle_ = transport::NodePtr(new transport::Node());
        node_handle_->Init(namespace_);

        getSdfParam<std::string>(sdf, "windPubTopic", wind_pub_topic_, wind_pub_topic_);
        double pub_rate = 2.0;
        getSdfParam<double>(sdf, "publishRate", pub_rate, pub_rate); //Wind topic publishing rates
        pub_interval_ = (pub_rate > 0.0) ? 1 / pub_rate : 0.0;
        getSdfParam<std::string>(sdf, "frameId", frame_id_, frame_id_);
        // Get the wind params from SDF.
        getSdfParam<double>(sdf, "windVelocityMean", wind_velocity_mean_, wind_velocity_mean_);
        getSdfParam<double>(sdf, "windVelocityMax", wind_velocity_max_, wind_velocity_max_);
        getSdfParam<double>(sdf, "windVelocityVariance", wind_velocity_variance_, wind_velocity_variance_);
        // Check if a custom static wind field should be used.
        getSdfParam<bool>(sdf, "useCustomStaticWindField", use_custom_static_wind_field_,
                          use_custom_static_wind_field_);

        if (!use_custom_static_wind_field_) {
            gzdbg << "[gazebo_wind_plugin] Using user-defined constant wind field and gusts.\n";
            // Get the wind params from SDF.
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
        } else {
            gzdbg << "[gazebo_wind_plugin] Using custom wind field from text file.\n";
            // Get the wind field text file path, read it and save data.
            std::string custom_wind_field_path;
            getSdfParam<std::string>(sdf, "linkName", link_name_, link_name_);
            getSdfParam<std::string>(sdf, "customWindFieldPath", custom_wind_field_path,
                                     custom_wind_field_path);

            ReadCustomWindField(custom_wind_field_path);
            const std::vector<gazebo::physics::ModelPtr> models = world->Models();
            for (auto model_: models) {
                gzdbg << "[gazebo_wind_plugin] Inspecting model: " << model_->GetName() << "." << std::endl;
                link_ = model_->GetLink(link_name_);
                if (link_ != NULL) {
                    gzdbg << "[gazebo_wind_plugin] " <<
                          model_->GetName() << " has link " << link_->GetName() << "." << std::endl;
                } else {
                    gzdbg << "[gazebo_wind_plugin] Model: " << model_->GetName() <<
                          "does not have link " << link_name_ << "." << std::endl;
                    //gzthrow("[gazebo_wind_plugin] Couldn't find specified link \"" << link_name_ << "\".");
                }
            }
        }

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboWindPlugin::OnUpdate, this, _1));

        wind_pub_ = node_handle_->Advertise<physics_msgs::msgs::Wind>("~/" + wind_pub_topic_, 10);

#if GAZEBO_MAJOR_VERSION >= 9
        last_time_ = world_->SimTime();
#else
        last_time_ = world_->GetSimTime();
#endif
    }

// This gets called by the world update start event.
    void GazeboWindPlugin::OnUpdate(const common::UpdateInfo &_info) {
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
        gazebo::msgs::Vector3d *wind_v;
        // Calculate the wind force.
        // Get normal distribution wind strength
        if (!use_custom_static_wind_field_) {
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

            wind_v = new gazebo::msgs::Vector3d();
            wind_v->set_x(wind.X() + wind_gust.X());
            wind_v->set_y(wind.Y() + wind_gust.Y());
            wind_v->set_z(wind.Z() + wind_gust.Z());
        } else {
            if (link_ == NULL) {
                const std::vector<gazebo::physics::ModelPtr> models = world_->Models();
                for (auto model_: models) {
                    gzdbg << "[gazebo_wind_plugin] Inspecting model: " << model_->GetName() << "." << std::endl;
                    link_ = model_->GetLink(link_name_);
                    if (link_ != NULL) {
                        gzdbg << "[gazebo_wind_plugin] " <<
                              model_->GetName() << " has link " << link_->GetName() << "." << std::endl;
                    } else {
                        gzdbg << "[gazebo_wind_plugin] Model: " << model_->GetName() <<
                              "does not have link " << link_name_ << "." << std::endl;
                        //gzthrow("[gazebo_wind_plugin] Couldn't find specified link \"" << link_name_ << "\".");
                    }
                }
            }
            if (link == NULL)
                return;
            ignition::math::Vector3d link_position = link_->WorldPose().Pos();
            const std::vector<gazebo::physics::ModelPtr> models = world_->Models();
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
//                std::cout << "idx[" << i << "]=" << ret_index[i] <<
//                          " (X,Y,Z) = (" << pt_cloud_vec3.pts[ret_index[i]].x << ", " <<
//                          pt_cloud_vec3.pts[ret_index[i]].y << ", " <<
//                          pt_cloud_vec3.pts[ret_index[i]].z << ") " << " dist[" << i
//                          << "]=" << out_dist_sqr[i] << std::endl;
                pt_idx = ret_index[i];
                wind_direction.X() +=
                        pt_cloud_vec3._data[pt_idx][0] * (1.0f / out_dist_sqr[i]) / total_invdistance_sqr;
                wind_direction.Y() +=
                        pt_cloud_vec3._data[pt_idx][1] * (1.0f / out_dist_sqr[i]) / total_invdistance_sqr;
                wind_direction.Z() +=
                        pt_cloud_vec3._data[pt_idx][2] * (1.0f / out_dist_sqr[i]) / total_invdistance_sqr;
                pt_cloud_vec3._data[pt_idx][0];
            }
            //std::cout << std::endl;

            double wind_strength = wind_direction.Length();
            if (wind_strength > wind_velocity_max_) {
                wind_direction *= wind_velocity_max_ / wind_strength;
            }
            // Get normal distribution wind direction
            ignition::math::Vector3d wind = wind_strength * wind_direction;
            wind_v = new gazebo::msgs::Vector3d();
            wind_v->set_x(wind.X());
            wind_v->set_y(wind.Y());
            wind_v->set_z(wind.Z());
        }
        wind_msg.set_frame_id(frame_id_);
        wind_msg.set_time_usec(now.Double() * 1e6);
        wind_msg.set_allocated_velocity(wind_v);

        wind_pub_->Publish(wind_msg);
    }

    void GazeboWindPlugin::CreatePubsAndSubs() {
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

    void GazeboWindPlugin::ReadCustomWindField(std::string &custom_wind_field_path) {
        std::ifstream fin;
        fin.open(custom_wind_field_path);
        if (fin.is_open()) {
            bool readCSV = true;
            if (readCSV) {
                // Create a vector to store the data
                std::vector<std::vector<double>> data;

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

                    while (getline(ss, value, ',')) {
                        double dvalue;
                        try {
                            dvalue = std::stod(value);
                        } catch (...) {
                            std::cout << "Could not convert string to double, skipping value on line " << linenum
                                      << std::endl;
                            std::cout << "line[" << linenum << "]=" << line << std::endl;
                            lineOK = false;
                            break;
                        }
                        row.push_back(dvalue);
                    }
                    if (lineOK) {
                        //std::cout << "(X,Y,Z), (U,V,W) = " <<
                        //          "(" << row[0] << ", " << row[1] << ", " << row[2] << "), " <<
                        //          "(" << row[3] << ", " << row[4] << ", " << row[5] << ")" << std::endl;
                        data.push_back(row);
                    }
                }

                // Generating point cloud of wind field vector measurements
                pt_cloud_vec3.pts.resize(data.size());
                pt_cloud_vec3._data.resize(data.size());
//                pt_cloud_vec3._data.push_back(Vector<3, float>());
                for (int i = 0; i < data.size(); i++) {
                    pt_cloud_vec3.pts[i].x = data[i][0];
                    pt_cloud_vec3.pts[i].y = data[i][1];
                    pt_cloud_vec3.pts[i].z = data[i][2];

                    pt_cloud_vec3._data[i][0] = data[i][3];
                    pt_cloud_vec3._data[i][1] = data[i][4];
                    pt_cloud_vec3._data[i][2] = data[i][5];
                    // Print the data
//                    std::cout << "(X,Y,Z), (U,V,W) = (" << pt_cloud_vec3.pts[i].x << ", " <<
//                              pt_cloud_vec3.pts[i].y << ", " << pt_cloud_vec3.pts[i].z << "), " <<
//                              "(" << pt_cloud_vec3._data[0][0] << ", " << pt_cloud_vec3._data[0][1] << ", " <<
//                              pt_cloud_vec3._data[0][3] << ")" << std::endl;
                }
//                using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
//                        nanoflann::L2_Simple_Adaptor<num_t, SampledVectorField<num_t, 3>>,
//                        SampledVectorField<num_t, 3>, 3 /* dim */>;
                //dump_mem_usage();
//                my_kd_tree_t windfield_kdtree(3, pt_cloud_vec3, {2 /* max elements in a leaf */});
                windfield_kdtree = new my_kd_tree_t(3, pt_cloud_vec3, {2 /* max elements in a leaf */});
                const num_t query_pt[3] = {100, 100, 20};
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

                    std::cout << "knnSearch(): num_results=" << num_results << std::endl;
                    for (size_t i = 0; i < num_results; i++)
                        std::cout << "idx[" << i << "]=" << ret_index[i] <<
                                  " (X,Y,Z) = (" << pt_cloud_vec3.pts[ret_index[i]].x << ", " <<
                                  pt_cloud_vec3.pts[ret_index[i]].y << ", " <<
                                  pt_cloud_vec3.pts[ret_index[i]].z << ") " << " dist[" << i
                                  << "]=" << out_dist_sqr[i] << std::endl;

                    std::cout << std::endl;
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
            } else {
                std::string data_name;
                float data;
                // Read the line with the variable name.
                while (fin >> data_name) {
                    // Save data on following line into the correct variable.
                    if (data_name == "min_x:") {
                        fin >> min_x_;
                    } else if (data_name == "min_y:") {
                        fin >> min_y_;
                    } else if (data_name == "n_x:") {
                        fin >> n_x_;
                    } else if (data_name == "n_y:") {
                        fin >> n_y_;
                    } else if (data_name == "res_x:") {
                        fin >> res_x_;
                    } else if (data_name == "res_y:") {
                        fin >> res_y_;
                    } else if (data_name == "vertical_spacing_factors:") {
                        while (fin >> data) {
                            vertical_spacing_factors_.push_back(data);
                            if (fin.peek() == '\n') break;
                        }
                    } else if (data_name == "bottom_z:") {
                        while (fin >> data) {
                            bottom_z_.push_back(data);
                            if (fin.peek() == '\n') break;
                        }
                    } else if (data_name == "top_z:") {
                        while (fin >> data) {
                            top_z_.push_back(data);
                            if (fin.peek() == '\n') break;
                        }
                    } else if (data_name == "u:") {
                        while (fin >> data) {
                            u_.push_back(data);
                            if (fin.peek() == '\n') break;
                        }
                    } else if (data_name == "v:") {
                        while (fin >> data) {
                            v_.push_back(data);
                            if (fin.peek() == '\n') break;
                        }
                    } else if (data_name == "w:") {
                        while (fin >> data) {
                            w_.push_back(data);
                            if (fin.peek() == '\n') break;
                        }
                    } else {
                        // If invalid data name, read the rest of the invalid line,
                        // publish a message and ignore data on next line. Then resume reading.
                        std::string restOfLine;
                        getline(fin, restOfLine);
                        gzerr << " [gazebo_wind_plugin] Invalid data name '" << data_name << restOfLine <<
                              "' in custom wind field text file. Ignoring data on next line.\n";
                        fin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    }
                }
            }
            fin.close();
            gzdbg << "[gazebo_wind_plugin] Successfully read custom wind field from text file.\n";
        } else {
            gzerr << "[gazebo_wind_plugin] Could not open custom wind field text file.\n";
        }
    }

    GZ_REGISTER_WORLD_PLUGIN(GazeboWindPlugin);
}
