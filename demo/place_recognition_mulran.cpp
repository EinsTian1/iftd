#include "include/IFTDesc.h"
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/filesystem.hpp>

// Read Mulran data
std::vector<float> read_lidar_data(const std::string lidar_data_path) {
  std::ifstream lidar_data_file;
  lidar_data_file.open(lidar_data_path,
                       std::ifstream::in | std::ifstream::binary);
  if (!lidar_data_file) {
    std::cout << "Read End..." << std::endl;
    std::vector<float> nan_data;
    return nan_data;
    // exit(-1);
  }
  lidar_data_file.seekg(0, std::ios::end);
  const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
  lidar_data_file.seekg(0, std::ios::beg);

  std::vector<float> lidar_data_buffer(num_elements);
  lidar_data_file.read(reinterpret_cast<char *>(&lidar_data_buffer[0]),
                       num_elements * sizeof(float));
  return lidar_data_buffer;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "demo_kitti");
  ros::NodeHandle nh;
  std::string lidar_path = "";
  std::string pose_path = "";
  std::string output_path = "";
  std::string config_path = "";
  nh.param<std::string>("lidar_path", lidar_path, "");
  nh.param<std::string>("pose_path", pose_path, "");
  nh.param<std::string>("output_path", output_path, "");

  ConfigSetting config_setting;
  read_parameters(nh, config_setting);
  sleep(3);
  ros::Publisher pubOdomAftMapped =
      nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
  // ros::Publisher pubRegisterCloud =
  //     nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
  ros::Publisher pubCureentCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_current", 100);
  ros::Publisher pubCurrentCorner =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_key_points", 100);
  ros::Publisher pubMatchedCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 100);
  ros::Publisher pubMatchedCorner =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched_key_points", 100);
  ros::Publisher pubSTD =
      nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);
  

  ros::Rate loop(500);
  ros::Rate slow_loop(10);

  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> poses_vec,poses_vec_;
  std::vector<double> times_vec,times_vec_;
  load_pose_with_time(pose_path, poses_vec_, times_vec_);
  std::cout << "Sucessfully load pose with number: " << poses_vec_.size()
            << std::endl;

  IFTDescManager *std_manager = new IFTDescManager(config_setting);
  int time_unit = std_manager->time_unit;

  size_t cloudInd = 0;
  size_t keyCloudInd = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());

  std::vector<double> descriptor_time;
  std::vector<double> querying_time;
  std::vector<double> update_time;
  int triggle_loop_num = 0;
  int num = 0;

  std::vector<std::string> bin_files_,bin_files;
  boost::filesystem::directory_iterator end_itr; // Default ctor yields past-the-end
  for (boost::filesystem::directory_iterator itr(lidar_path); itr != end_itr; ++itr)
  {
    if (itr->path().extension() == ".bin")
    {
      bin_files_.push_back(itr->path().stem().string());
    }
  }
  auto compare = [](const std::string &a, const std::string &b)
  {
    long long int num_a = std::stoll(a);
    long long int num_b = std::stoll(b);
    return num_a < num_b;
  };
  std::sort(bin_files_.begin(), bin_files_.end(), compare);

  pose_bin_timestamp_align(bin_files_,bin_files, poses_vec, times_vec, poses_vec_, times_vec_, time_unit);
  std::cout << "bin_files_ size: " << bin_files_.size() << " poses_vec_ size: " << poses_vec_.size() << std::endl;
  std::cout << "bin_files size: " << bin_files.size() << " poses_vec size: " << poses_vec.size() << std::endl;

  while (ros::ok()) {
    std::stringstream lidar_data_path;
    lidar_data_path << lidar_path  <<  bin_files[cloudInd] << ".bin";
    std::cout << lidar_data_path.str() << std::endl;
    std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());
    Eigen::Vector3d translation_l2b(1.77, 0.0, -0.05);
    Eigen::Matrix3d rotation_l2b;
    rotation_l2b << -1, 0, 0, 0, -1, 0, 0, 0, 1;

    if (lidar_data.size() == 0) {
      break;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(
        new pcl::PointCloud<pcl::PointXYZI>());
    Eigen::Vector3d translation = poses_vec[cloudInd].first;
    Eigen::Matrix3d rotation = poses_vec[cloudInd].second;

    for (std::size_t i = 0; i < lidar_data.size(); i += 4) {
      pcl::PointXYZI point;
      point.x = lidar_data[i];
      point.y = lidar_data[i + 1];
      point.z = lidar_data[i + 2];
      point.intensity = lidar_data[i + 3];

      Eigen::Vector3d pv = point2vec(point);
      pv = rotation_l2b * pv + translation_l2b;
      pv = rotation * pv + translation;
      point = vec2point(pv);
      current_cloud->push_back(point);
    }

    down_sampling_voxel(*current_cloud, config_setting.ds_size_);
    for (auto pv : current_cloud->points) {
      temp_cloud->points.push_back(pv);
    }

    // check if keyframe
    if (cloudInd % config_setting.sub_frame_num_ == 0 && cloudInd != 0) {
      std::cout << "Key Frame id:" << keyCloudInd
                << ", cloud size: " << temp_cloud->size() << std::endl;

      // step1. Descriptor Extraction
      std::cout << "Descriptor Extraction..." << std::endl;
      auto t_descriptor_begin = std::chrono::high_resolution_clock::now();
      std::vector<IFTDesc> stds_vec;
      std_manager->GenerateIFTDescs(temp_cloud, stds_vec, num, translation, rotation);
      num++;
      auto t_descriptor_end = std::chrono::high_resolution_clock::now();
      descriptor_time.push_back(time_inc(t_descriptor_end, t_descriptor_begin));

      // step2. Searching Loop
      std::cout << "Searching Loop..." << std::endl;
      auto t_query_begin = std::chrono::high_resolution_clock::now();
      std::pair<int, double> search_result(-1, 0);
      std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
      loop_transform.first << 0, 0, 0;
      loop_transform.second = Eigen::Matrix3d::Identity();
      std::vector<std::pair<IFTDesc, IFTDesc>> loop_std_pair;
      if (keyCloudInd > config_setting.skip_near_num_) {
        std_manager->SearchLoop(stds_vec, search_result, loop_transform,
                                loop_std_pair);
      }
      if (search_result.first > 0) {
        std::cout << "[Loop Detection] triggle loop: " << keyCloudInd << "--"
                  << search_result.first << ", score:" << search_result.second
                  << std::endl;
      }

      double cloudIndDouble = static_cast<double>(std::stod(bin_files[cloudInd])/time_unit);
      std::ofstream foutC(std::string(output_path + "/loop.txt"), std::ios::app);
      foutC << std::fixed << cloudIndDouble << " " << keyCloudInd << " " << search_result.first << std::endl;
      foutC.close();

      auto t_query_end = std::chrono::high_resolution_clock::now();
      querying_time.push_back(time_inc(t_query_end, t_query_begin));

      // step3. Add descriptors to the database
      std::cout << "Add descriptors to the database..." << std::endl;
      auto t_map_update_begin = std::chrono::high_resolution_clock::now();
      std_manager->AddIFTDescs(stds_vec);
      auto t_map_update_end = std::chrono::high_resolution_clock::now();
      update_time.push_back(time_inc(t_map_update_end, t_map_update_begin));
      std::cout << "[Time] descriptor extraction: "
                << time_inc(t_descriptor_end, t_descriptor_begin) << "ms, "
                << "query: " << time_inc(t_query_end, t_query_begin) << "ms, "
                << "update map:"
                << time_inc(t_map_update_end, t_map_update_begin) << "ms"
                << std::endl;
      std::cout << std::endl;

      pcl::PointCloud<pcl::PointXYZI> save_key_cloud;
      save_key_cloud = *temp_cloud;

      std_manager->key_cloud_vec_.push_back(save_key_cloud.makeShared());

      // // publish
      std::cout << "publish..." << std::endl;
      sensor_msgs::PointCloud2 pub_cloud;
      pcl::toROSMsg(*temp_cloud, pub_cloud);
      pub_cloud.header.frame_id = "camera_init";
      pubCureentCloud.publish(pub_cloud);
      pcl::toROSMsg(*std_manager->corner_cloud_vec_.back(), pub_cloud);
      pub_cloud.header.frame_id = "camera_init";
      pubCurrentCorner.publish(pub_cloud);

      if (search_result.first > 0) {
        triggle_loop_num++;
        pcl::toROSMsg(*std_manager->key_cloud_vec_[search_result.first],
                      pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubMatchedCloud.publish(pub_cloud);
        slow_loop.sleep();
        pcl::toROSMsg(*std_manager->corner_cloud_vec_[search_result.first],
                      pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubMatchedCorner.publish(pub_cloud);
        publish_std_pairs(loop_std_pair, translation, rotation,pubSTD);
        slow_loop.sleep();
        // getchar();
      }
 
      loop.sleep();     
      temp_cloud->clear();
      keyCloudInd++;
    }
    nav_msgs::Odometry odom;
    odom.header.frame_id = "camera_init";
    odom.pose.pose.position.x = translation[0];
    odom.pose.pose.position.y = translation[1];
    odom.pose.pose.position.z = translation[2];
    Eigen::Quaterniond q(rotation);
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    pubOdomAftMapped.publish(odom);
    loop.sleep();
    cloudInd++;
    if(cloudInd == bin_files.size())
    {
      break;
    }

  }
  double mean_descriptor_time =
      std::accumulate(descriptor_time.begin(), descriptor_time.end(), 0) * 1.0 /
      descriptor_time.size();
  double mean_query_time =
      std::accumulate(querying_time.begin(), querying_time.end(), 0) * 1.0 /
      querying_time.size();
  double mean_update_time =
      std::accumulate(update_time.begin(), update_time.end(), 0) * 1.0 /
      update_time.size();
  std::cout << "Total key frame number:" << keyCloudInd
            << ", loop number:" << triggle_loop_num << std::endl;
  std::cout << "Mean time for descriptor extraction: " << mean_descriptor_time
            << "ms, query: " << mean_query_time
            << "ms, update: " << mean_update_time << "ms, total: "
            << mean_descriptor_time + mean_query_time + mean_update_time << "ms"
            << std::endl;
  return 0;
}