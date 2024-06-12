#pragma once

#include "omp.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <fstream>
#include <mutex>
#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sstream>
#include <stdio.h>
#include <string>
#include <unordered_map>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <bitset>

#define HASH_P 116101
#define MAX_N 10000000000
#define MAX_FRAME_N 20000
extern std::string output_path;

typedef struct ConfigSetting {
  /* for point cloud pre-preocess*/
  double ds_size_ = 0.5;
  /* for bin_timestamp */
  int bin_timestamp = 1;
  /* for BEV */
  int BEV_X_NUM = 400; 
  int BEV_Y_NUM = 400;
  int BEV_X_MAX = 60;
  int BEV_Y_MAX = 60;
  double lidar_height = 1.5;
  double height_bin = 0.3;
  /* feature extraction */
  double image_quartity = 0.2;
  int Hamming_distance = 10;
  /* for STD */
  int descriptor_near_num_ = 15;
  double descriptor_min_len_ = 2;
  double descriptor_max_len_ = 50;
  double std_side_resolution_ = 0.2;

  /* for place recognition*/
  int skip_near_num_ = 50;
  int candidate_num_ = 50;
  int sub_frame_num_ = 10;
  double rough_dis_threshold_ = 0.4;
  double vertex_diff_threshold_ = 0.01;
  int image_reseize_ = 20;
  double distance_threshold_ = 15;
  double image_threshold_ = 0.2;

} ConfigSetting;

// Structure for Stabel Triangle Descriptor
typedef struct IFTDesc {
  // the side lengths of IFTDesc, arranged from short to long
  Eigen::Vector3d side_length_;

  // projection angle between vertices
  Eigen::Vector3d angle_;

  Eigen::Vector3d center_;
  unsigned int frame_id_;

  // three vertexs
  Eigen::Vector3d vertex_A_;
  Eigen::Vector3d vertex_B_;
  Eigen::Vector3d vertex_C_;

  // some other inform attached to each vertex,e.g., intensity
  Eigen::Vector3d vertex_attached_;
} IFTDesc;

// plane structure for corner point extraction
typedef struct Plane {
  pcl::PointXYZINormal p_center_;
  Eigen::Vector3d center_;
  Eigen::Vector3d normal_;
  Eigen::Matrix3d covariance_;
  float radius_ = 0;
  float min_eigen_value_ = 1;
  float intercept_ = 0;
  int id_ = 0;
  int sub_plane_num_ = 0;
  int points_size_ = 0;
  bool is_plane_ = false;
} Plane;

typedef struct STDMatchList {
  std::vector<std::pair<IFTDesc, IFTDesc>> match_list_;
  std::pair<int, int> match_id_;
  double mean_dis_;
} STDMatchList;

class VOXEL_LOC {
public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
      : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

// for down sample function
struct M_POINT {
  float xyz[3];
  float intensity;
  int count = 0;
};

// Hash value

template <> struct std::hash<VOXEL_LOC> {
  int64_t operator()(const VOXEL_LOC &s) const {
    using std::hash;
    using std::size_t;
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
  }
};

class IFTDesc_LOC {
public:
  int64_t x, y, z, a, b, c;

  IFTDesc_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0, int64_t va = 0,
             int64_t vb = 0, int64_t vc = 0)
      : x(vx), y(vy), z(vz), a(va), b(vb), c(vc) {}

  bool operator==(const IFTDesc_LOC &other) const {
    // use three attributes
    return (x == other.x && y == other.y && z == other.z);
    // use six attributes
    // return (x == other.x && y == other.y && z == other.z && a == other.a &&
    //         b == other.b && c == other.c);
  }
};

template <> struct std::hash<IFTDesc_LOC> {
  int64_t operator()(const IFTDesc_LOC &s) const {
    using std::hash;
    using std::size_t;
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
    // return ((((((((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N +
    //              (s.x)) *
    //             HASH_P) %
    //                MAX_N +
    //            s.a) *
    //           HASH_P) %
    //              MAX_N +
    //          s.b) *
    //         HASH_P) %
    //            MAX_N +
    //        s.c;
  }
};

void down_sampling_voxel(pcl::PointCloud<pcl::PointXYZI> &pl_feat,
                         double voxel_size);

void load_pose_with_time(
    const std::string &pose_file,
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> &poses_vec,
    std::vector<double> &times_vec);

  void pose_bin_timestamp_align(std::vector<std::string> bin_files,std::vector<std::string> &outbin_files,
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> &outposes_vec,std::vector<double> &outtimes_vec,
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> &inposes_vec,std::vector<double> &intimes_vec,
    int time_unit);

void read_parameters(ros::NodeHandle &nh, ConfigSetting &config_setting);

double time_inc(std::chrono::_V2::system_clock::time_point &t_end,
                std::chrono::_V2::system_clock::time_point &t_begin);

pcl::PointXYZI vec2point(const Eigen::Vector3d &vec);
Eigen::Vector3d point2vec(const pcl::PointXYZI &pi);

void publish_std_pairs(
    const std::vector<std::pair<IFTDesc, IFTDesc>> &match_std_pairs,Eigen::Vector3d translation ,Eigen::Matrix3d rotation, 
    const ros::Publisher &std_publisher);

bool attach_greater_sort(std::pair<double, int> a, std::pair<double, int> b);

class IFTDescManager {
public:
  IFTDescManager() = default;

  ConfigSetting config_setting_;

  unsigned int current_frame_id_;

  IFTDescManager(ConfigSetting &config_setting)
      : config_setting_(config_setting) {
    current_frame_id_ = 0;
    if(config_setting_.bin_timestamp == 1)
      time_unit = 1e0;
    else if(config_setting_.bin_timestamp == 2)
      time_unit = 1e3;
    else if(config_setting_.bin_timestamp == 3)
      time_unit = 1e6;
    else
      time_unit = 1e9;
    
  
    BEV_X_NUM = config_setting.BEV_X_NUM;
    BEV_Y_NUM = config_setting.BEV_Y_NUM;
    BEV_X_MAX = config_setting.BEV_X_MAX;
    BEV_Y_MAX = config_setting.BEV_Y_MAX;
    lidar_height = config_setting.lidar_height;
    height_bin = config_setting.height_bin;
    image_quartity = config_setting.image_quartity;
    Hamming_distance = config_setting.Hamming_distance;
  };

  int time_unit;
  int BEV_X_NUM;
  int BEV_Y_NUM; 
  double BEV_X_MAX;
  double BEV_Y_MAX; 
  double lidar_height;
  double height_bin;
  double image_quartity;
  int Hamming_distance;

  std::vector<std::pair <double, double>> centors;
  std::vector<cv::Mat> BEV_images;
  std::vector<std::pair<int,int>> loop_pairs;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> relative_poses;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> world_poses;


  //time co
  std::vector<double> candidate_selector_time;
  std::vector<double> candidate_verify_time;
  // hash table, save all descriptors
  std::unordered_map<IFTDesc_LOC, std::vector<IFTDesc>> data_base_;

  // save all key clouds, optional
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> key_cloud_vec_;

  // save all corner points, optional
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> corner_cloud_vec_;

  // save all planes of key frame, required
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> plane_cloud_vec_;

  /*Three main processing functions*/

  // generate IFTDescs from a point cloud
  void GenerateIFTDescs(pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                       std::vector<IFTDesc> &stds_vec, int num ,Eigen::Vector3d translation ,Eigen::Matrix3d rotation);

  // search result <candidate_id, plane icp score>. -1 for no loop
  void SearchLoop(const std::vector<IFTDesc> &stds_vec,
                  std::pair<int, double> &loop_result,
                  std::pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform,
                  std::vector<std::pair<IFTDesc, IFTDesc>> &loop_std_pair);

  // add descriptors to database
  void AddIFTDescs(const std::vector<IFTDesc> &stds_vec);

private:
  // build IFTDescs from corner points.
  void
  build_IFTDesc(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &corner_points,
               std::vector<IFTDesc> &stds_vec);

  // Select a specified number of candidate frames according to the number of
  // IFTDesc rough matches
  void candidate_selector(const std::vector<IFTDesc> &stds_vec,
                          std::vector<STDMatchList> &candidate_matcher_vec);

  // Get the best candidate frame by geometry check
  void
  candidate_verify(const STDMatchList &candidate_matcher, double &verify_score,
                   std::pair<Eigen::Vector3d, Eigen::Matrix3d> &relative_pose,
                   std::vector<std::pair<IFTDesc, IFTDesc>> &sucess_match_vec);

  // Get the transform between a matched std pair
  void triangle_solver(std::pair<IFTDesc, IFTDesc> &std_pair, Eigen::Vector3d &t,
                       Eigen::Matrix3d &rot);
  //
  std::pair<Eigen::MatrixXd ,Eigen::MatrixXi> makeBEV(pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud, Eigen::Vector3d translation, Eigen::Matrix3d rotation);
  cv::Mat Eigen2Mat(Eigen::MatrixXi &matrix);
  double image_similarity_verify(int source_frame_id, int target_frame_id, const Eigen::Matrix3d &rot,const Eigen::Vector3d &t);
  cv::Mat image_transformer(cv::Mat &src, cv::Mat &transform_matrix);
  void vertex_test(Eigen::Vector3d A,std::vector<Eigen::Vector3d> &vertexs);
  Eigen::VectorXd dhash(cv::Mat image);
};