/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef COMMON_LIB_H
#define COMMON_LIB_H
#define PCL_NO_PRECOMPILE

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "color.h"

using namespace std;
using namespace cv;
using namespace pcl;

#define TARGET_NUM_CIRCLES 4
#define DEBUG 1
#define GEOMETRY_TOLERANCE 0.08

namespace Common 
{
  struct Point
  {
    PCL_ADD_POINT4D;
    std::uint16_t ring = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;
}
POINT_CLOUD_REGISTER_POINT_STRUCT(Common::Point,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (std::uint16_t, ring, ring)
);

struct Params {
  double x_min, x_max, y_min, y_max, z_min, z_max;
  double fx, fy, cx, cy, k1, k2, p1, p2;
  double marker_size, delta_width_qr_center, delta_height_qr_center;
  double delta_width_circles, delta_height_circles, circle_radius;
  int min_detected_markers;
  string image_path;
  string bag_path;
  string lidar_topic;
  string image_topic;
  string output_path;
};

inline Params loadParameters(rclcpp::Node::SharedPtr node) {
  Params params;

  auto declare = [&](const std::string& name, auto default_val) {
    node->declare_parameter(name, default_val);
  };

  declare("fx", 1215.31801774424);
  declare("fy", 1214.72961288138);
  declare("cx", 1047.86571859677);
  declare("cy", 745.068353101898);
  declare("k1", -0.33574781188503);
  declare("k2", 0.10996870793601);
  declare("p1", 0.000157303079833973);
  declare("p2", 0.000544930726278493);
  declare("marker_size", 0.2);
  declare("delta_width_qr_center", 0.55);
  declare("delta_height_qr_center", 0.35);
  declare("delta_width_circles", 0.5);
  declare("delta_height_circles", 0.4);
  declare("min_detected_markers", 3);
  declare("circle_radius", 0.12);
  declare("image_path", std::string(""));
  declare("bag_path", std::string(""));
  declare("lidar_topic", std::string("/rslidar_points"));
  declare("image_topic", std::string("/camera/image_raw"));
  declare("output_path", std::string("output"));
  declare("x_min", 1.5);
  declare("x_max", 3.0);
  declare("y_min", -1.5);
  declare("y_max", 2.0);
  declare("z_min", -0.5);
  declare("z_max", 2.0);

  node->get_parameter("fx", params.fx);
  node->get_parameter("fy", params.fy);
  node->get_parameter("cx", params.cx);
  node->get_parameter("cy", params.cy);
  node->get_parameter("k1", params.k1);
  node->get_parameter("k2", params.k2);
  node->get_parameter("p1", params.p1);
  node->get_parameter("p2", params.p2);
  node->get_parameter("marker_size", params.marker_size);
  node->get_parameter("delta_width_qr_center", params.delta_width_qr_center);
  node->get_parameter("delta_height_qr_center", params.delta_height_qr_center);
  node->get_parameter("delta_width_circles", params.delta_width_circles);
  node->get_parameter("delta_height_circles", params.delta_height_circles);
  node->get_parameter("min_detected_markers", params.min_detected_markers);
  node->get_parameter("circle_radius", params.circle_radius);
  node->get_parameter("image_path", params.image_path);
  node->get_parameter("bag_path", params.bag_path);
  node->get_parameter("lidar_topic", params.lidar_topic);
  node->get_parameter("image_topic", params.image_topic);
  node->get_parameter("output_path", params.output_path);
  node->get_parameter("x_min", params.x_min);
  node->get_parameter("x_max", params.x_max);
  node->get_parameter("y_min", params.y_min);
  node->get_parameter("y_max", params.y_max);
  node->get_parameter("z_min", params.z_min);
  node->get_parameter("z_max", params.z_max);

  return params;
}

inline double computeRMSE(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1, 
                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2) 
{
    if (cloud1->size() != cloud2->size()) 
    {
      std::cerr << BOLDRED << "[computeRMSE] Point cloud sizes do not match, cannot compute RMSE." << RESET << std::endl;
      return -1.0;
    }

    double sum = 0.0;
    for (size_t i = 0; i < cloud1->size(); ++i) 
    {
      double dx = cloud1->points[i].x - cloud2->points[i].x;
      double dy = cloud1->points[i].y - cloud2->points[i].y;
      double dz = cloud1->points[i].z - cloud2->points[i].z;

      sum += dx * dx + dy * dy + dz * dz;
    }

    double mse = sum / cloud1->size();
    return std::sqrt(mse);
}

inline void alignPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud, const Eigen::Matrix4f &transformation) 
{
  output_cloud->clear();
  for (const auto &pt : input_cloud->points) 
  {
    Eigen::Vector4f pt_homogeneous(pt.x, pt.y, pt.z, 1.0);
    Eigen::Vector4f transformed_pt = transformation * pt_homogeneous;
    output_cloud->push_back(pcl::PointXYZ(transformed_pt(0), transformed_pt(1), transformed_pt(2)));
  }
}

inline void comb(int N, int K, std::vector<std::vector<int>> &groups) {
  int upper_factorial = 1;
  int lower_factorial = 1;

  for (int i = 0; i < K; i++) {
    upper_factorial *= (N - i);
    lower_factorial *= (K - i);
  }
  int n_permutations = upper_factorial / lower_factorial;

  if (DEBUG)
    cout << N << " centers found. Iterating over " << n_permutations
         << " possible sets of candidates" << endl;

  std::string bitmask(K, 1);
  bitmask.resize(N, 0);

  do {
    std::vector<int> group;
    for (int i = 0; i < N; ++i)
    {
      if (bitmask[i]) {
        group.push_back(i);
      }
    }
    groups.push_back(group);
  } while (std::prev_permutation(bitmask.begin(), bitmask.end()));

  assert(static_cast<int>(groups.size()) == n_permutations);
}

inline void projectPointCloudToImage(const pcl::PointCloud<Common::Point>::Ptr& cloud,
  const Eigen::Matrix4f& transformation,
  const cv::Mat& cameraMatrix,
  const cv::Mat& distCoeffs,
  const cv::Mat& image,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud) 
{
  colored_cloud->clear();
  colored_cloud->reserve(cloud->size());

  cv::Mat undistortedImage;
  cv::undistort(image, undistortedImage, cameraMatrix, distCoeffs);

  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32F);
  cv::Mat zeroDistCoeffs = cv::Mat::zeros(5, 1, CV_32F);

  std::vector<cv::Point3f> objectPoints(1);
  std::vector<cv::Point2f> imagePoints(1);

  for (const auto& point : *cloud) 
  {
    Eigen::Vector4f homogeneous_point(point.x, point.y, point.z, 1.0f);
    Eigen::Vector4f transformed_point = transformation * homogeneous_point;

    if (transformed_point(2) < 0) continue;

    objectPoints[0] = cv::Point3f(transformed_point(0), transformed_point(1), transformed_point(2));
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, zeroDistCoeffs, imagePoints);

    int u = static_cast<int>(imagePoints[0].x);
    int v = static_cast<int>(imagePoints[0].y);

    if (u >= 0 && u < undistortedImage.cols && v >= 0 && v < undistortedImage.rows) 
    {
      cv::Vec3b color = undistortedImage.at<cv::Vec3b>(v, u);

      pcl::PointXYZRGB colored_point;
      colored_point.x = transformed_point(0);
      colored_point.y = transformed_point(1);
      colored_point.z = transformed_point(2);
      colored_point.r = color[2];
      colored_point.g = color[1];
      colored_point.b = color[0];
      colored_cloud->push_back(colored_point);
    }
  }
}

inline void saveTargetHoleCenters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& lidar_centers,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& qr_centers,
                      const Params& params)
{
    if (lidar_centers->size() != 4 || qr_centers->size() != 4) {
      std::cerr << "[saveTargetHoleCenters] The number of points in lidar_centers or qr_centers is not 4, skip saving." << std::endl;
      return;
    }
    
    std::string saveDir = params.output_path;
    if (saveDir.back() != '/') saveDir += '/';
    std::ofstream saveFile(saveDir + "circle_center_record.txt", std::ios::app);

    if (!saveFile.is_open()) {
        std::cerr << "[saveTargetHoleCenters] Cannot open file: " << saveDir + "circle_center_record.txt" << std::endl;
        return;
    }

    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    saveFile << "time: " << std::put_time(std::localtime(&now_time), "%Y-%m-%d %H:%M:%S") << std::endl;

    saveFile << "lidar_centers:";
    for (const auto& pt : lidar_centers->points) {
        saveFile << " {" << pt.x << "," << pt.y << "," << pt.z << "}";
    }
    saveFile << std::endl;
    saveFile << "qr_centers:";
    for (const auto& pt : qr_centers->points) {
        saveFile << " {" << pt.x << "," << pt.y << "," << pt.z << "}";
    }
    saveFile << std::endl;
    saveFile.close();
    std::cout << BOLDGREEN << "[Record] Saved four pairs of circular hole centers to " << BOLDWHITE << saveDir << "circle_center_record.txt" << RESET << std::endl;
}

inline void saveCalibrationResults(const Params& params, const Eigen::Matrix4f& transformation, 
     const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud, const cv::Mat& img_input)
{
  if(colored_cloud->empty()) 
  {
    std::cerr << BOLDRED << "[saveCalibrationResults] Colored point cloud is empty!" << RESET << std::endl;
    return;
  }
  std::string outputDir = params.output_path;
  if (outputDir.back() != '/') outputDir += '/';

  std::ofstream outFile(outputDir + "single_calib_result.txt");
  if (outFile.is_open()) 
  {
    outFile << "# FAST-LIVO2 calibration format\n";
    outFile << "cam_model: Pinhole\n";
    outFile << "cam_width: " << img_input.cols << "\n";
    outFile << "cam_height: " << img_input.rows << "\n";
    outFile << "scale: 1.0\n";
    outFile << "cam_fx: " << params.fx << "\n";
    outFile << "cam_fy: " << params.fy << "\n";
    outFile << "cam_cx: " << params.cx << "\n";
    outFile << "cam_cy: " << params.cy << "\n";
    outFile << "cam_d0: " << params.k1 << "\n";
    outFile << "cam_d1: " << params.k2 << "\n";
    outFile << "cam_d2: " << params.p1 << "\n";
    outFile << "cam_d3: " << params.p2 << "\n";

    outFile << "\nRcl: [" << std::fixed << std::setprecision(6);
    outFile << std::setw(10) << transformation(0, 0) << ", " << std::setw(10) << transformation(0, 1) << ", " << std::setw(10) << transformation(0, 2) << ",\n";
    outFile << "      " << std::setw(10) << transformation(1, 0) << ", " << std::setw(10) << transformation(1, 1) << ", " << std::setw(10) << transformation(1, 2) << ",\n";
    outFile << "      " << std::setw(10) << transformation(2, 0) << ", " << std::setw(10) << transformation(2, 1) << ", " << std::setw(10) << transformation(2, 2) << "]\n";

    outFile << "Pcl: [";
    outFile << std::setw(10) << transformation(0, 3) << ", " << std::setw(10) << transformation(1, 3) << ", " << std::setw(10) << transformation(2, 3) << "]\n";

    outFile.close();
    std::cout << BOLDYELLOW << "[Result] Single-scene calibration results saved to " << BOLDWHITE << outputDir << "single_calib_result.txt" << RESET << std::endl;
  } 
  else
  {
    std::cerr << BOLDRED << "[Error] Failed to open single_calib_result.txt for writing!" << RESET << std::endl;
  }
  
  if (pcl::io::savePCDFileASCII(outputDir + "colored_cloud.pcd", *colored_cloud) == 0) 
  {
    std::cout << BOLDYELLOW << "[Result] Saved colored point cloud to: " << BOLDWHITE << outputDir << "colored_cloud.pcd" << RESET << std::endl;
  } 
  else 
  {
    std::cerr << BOLDRED << "[Error] Failed to save colored point cloud to " << outputDir << "colored_cloud.pcd" << "!" << RESET << std::endl;
  }
 
  imwrite(outputDir + "qr_detect.png", img_input);
}

inline void sortPatternCenters(pcl::PointCloud<pcl::PointXYZ>::Ptr pc,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr v,
                        const std::string& axis_mode = "camera") 
{
  if (pc->size() != 4) {
    std::cerr << BOLDRED << "[sortPatternCenters] Number of " << axis_mode << " center points to be sorted is not 4." << RESET << std::endl;
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr work_pc(new pcl::PointCloud<pcl::PointXYZ>());

  if (axis_mode == "lidar") {
    for (const auto& p : *pc) {
      pcl::PointXYZ pt;
      pt.x = -p.y;
      pt.y = -p.z;
      pt.z = p.x;
      work_pc->push_back(pt);
    }
  } else {
    *work_pc = *pc;
  }

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*work_pc, centroid);
  pcl::PointXYZ ref_origin(centroid[0], centroid[1], centroid[2]);

  std::vector<std::pair<float, int>> proj_points;
  for (size_t i = 0; i < work_pc->size(); ++i) {
    const auto& p = work_pc->points[i];
    Eigen::Vector3f rel_vec(p.x - ref_origin.x, p.y - ref_origin.y, p.z - ref_origin.z);
    proj_points.emplace_back(atan2(rel_vec.y(), rel_vec.x()), i);
  }

  std::sort(proj_points.begin(), proj_points.end());

  v->resize(4);
  for (int i = 0; i < 4; ++i) {
    (*v)[i] = work_pc->points[proj_points[i].second];
  }

  const auto& p0 = v->points[0];
  const auto& p1 = v->points[1];
  const auto& p2 = v->points[2];
  Eigen::Vector3f v01(p1.x - p0.x, p1.y - p0.y, 0);
  Eigen::Vector3f v12(p2.x - p1.x, p2.y - p1.y, 0);
  if (v01.cross(v12).z() > 0) {
    std::swap((*v)[1], (*v)[3]);
  }

  if (axis_mode == "lidar") {
    for (auto& point : v->points) {
      float x_new = point.z;
      float y_new = -point.x;
      float z_new = -point.y;
      point.x = x_new;
      point.y = y_new;
      point.z = z_new;
    }
  }
}

class Square 
{
  private:
    pcl::PointXYZ _center;
    std::vector<pcl::PointXYZ> _candidates;
    float _target_width, _target_height, _target_diagonal;
 
  public:
    Square(std::vector<pcl::PointXYZ> candidates, float width, float height) {
      _candidates = candidates;
      _target_width = width;
      _target_height = height;
      _target_diagonal = sqrt(pow(width, 2) + pow(height, 2));
 
      _center.x = _center.y = _center.z = 0;
      for (size_t i = 0; i < candidates.size(); ++i) {
        _center.x += candidates[i].x;
        _center.y += candidates[i].y;
        _center.z += candidates[i].z;
      }
 
      _center.x /= candidates.size();
      _center.y /= candidates.size();
      _center.z /= candidates.size();
    }
 
    float distance(pcl::PointXYZ pt1, pcl::PointXYZ pt2) {
      return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) +
                  pow(pt1.z - pt2.z, 2));
    }
 
    pcl::PointXYZ at(int i) {
      assert(0 <= i && i < 4);
      return _candidates[i];
    }
 
    bool is_valid() 
    {
      if (_candidates.size() != 4) return false;

      pcl::PointCloud<pcl::PointXYZ>::Ptr candidates_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      for(const auto& p : _candidates) candidates_cloud->push_back(p);

      for (size_t i = 0; i < _candidates.size(); ++i) {
        float d = distance(_center, _candidates[i]);
        if (fabs(d - _target_diagonal / 2.) / (_target_diagonal / 2.) > GEOMETRY_TOLERANCE * 2.0) {
          return false;
        }
      }
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_centers(new pcl::PointCloud<pcl::PointXYZ>());
      sortPatternCenters(candidates_cloud, sorted_centers, "camera");
      
      float s01 = distance(sorted_centers->points[0], sorted_centers->points[1]);
      float s12 = distance(sorted_centers->points[1], sorted_centers->points[2]);
      float s23 = distance(sorted_centers->points[2], sorted_centers->points[3]);
      float s30 = distance(sorted_centers->points[3], sorted_centers->points[0]);

      bool pattern1_ok = 
        (fabs(s01 - _target_width) / _target_width < GEOMETRY_TOLERANCE) &&
        (fabs(s12 - _target_height) / _target_height < GEOMETRY_TOLERANCE) &&
        (fabs(s23 - _target_width) / _target_width < GEOMETRY_TOLERANCE) &&
        (fabs(s30 - _target_height) / _target_height < GEOMETRY_TOLERANCE);

      bool pattern2_ok = 
        (fabs(s01 - _target_height) / _target_height < GEOMETRY_TOLERANCE) &&
        (fabs(s12 - _target_width) / _target_width < GEOMETRY_TOLERANCE) &&
        (fabs(s23 - _target_height) / _target_height < GEOMETRY_TOLERANCE) &&
        (fabs(s30 - _target_width) / _target_width < GEOMETRY_TOLERANCE);

      if (!pattern1_ok && !pattern2_ok) {
        return false;
      }
      
      float perimeter = s01 + s12 + s23 + s30;
      float ideal_perimeter = 2 * (_target_width + _target_height);
      if (fabs(perimeter - ideal_perimeter) / ideal_perimeter > GEOMETRY_TOLERANCE) {
        return false;
      }
 
      return true;
    }
};

#endif
