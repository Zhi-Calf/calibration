/*
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "qr_detect.hpp"
#include "lidar_detect.hpp"
#include "data_preprocess.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fast_calib");

  Params params = loadParameters(node);

  QRDetectPtr qrDetectPtr;
  qrDetectPtr.reset(new QRDetect(node, params));

  LidarDetectPtr lidarDetectPtr;
  lidarDetectPtr.reset(new LidarDetect(node, params));

  DataPreprocessPtr dataPreprocessPtr;
  dataPreprocessPtr.reset(new DataPreprocess(params, node->get_logger()));

  cv::Mat img_input = dataPreprocessPtr->img_input_;
  pcl::PointCloud<Common::Point>::Ptr cloud_input = dataPreprocessPtr->cloud_input_;
  const bool apriltag_mode = (params.target_mode == "apriltag_grid");
  const bool need_image_pick_first = (apriltag_mode && params.use_image_pick);
  const int required_picks = apriltag_mode ? 4 : 1;

  auto loadLidarPoints = [&](const std::string &file_path,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) -> bool {
    cloud->clear();
    if (file_path.empty()) return false;
    std::ifstream fin(file_path);
    if (!fin.is_open()) return false;

    std::string line;
    while (std::getline(fin, line)) {
      if (line.empty()) continue;
      std::istringstream iss(line);
      pcl::PointXYZ p;
      if (!(iss >> p.x >> p.y >> p.z)) continue;
      cloud->push_back(p);
    }
    if (static_cast<int>(cloud->size()) != required_picks) {
      RCLCPP_WARN(node->get_logger(),
                  "[LidarAutoPick] points in %s = %zu, expected %d. Ignore and fallback to manual pick.",
                  file_path.c_str(), cloud->size(), required_picks);
      cloud->clear();
      return false;
    }
    return true;
  };

  auto saveLidarPoints = [&](const std::string &file_path,
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    if (file_path.empty()) return;
    if (cloud->empty()) return;
    std::filesystem::path out(file_path);
    if (!out.parent_path().empty()) {
      std::error_code ec;
      std::filesystem::create_directories(out.parent_path(), ec);
      if (ec) {
        RCLCPP_WARN(node->get_logger(),
                    "[LidarAutoPick] failed to create parent dir for %s: %s",
                    file_path.c_str(), ec.message().c_str());
      }
    }
    std::ofstream fout(file_path);
    if (!fout.is_open()) {
      RCLCPP_WARN(node->get_logger(),
                  "[LidarAutoPick] failed to open %s for write.",
                  file_path.c_str());
      return;
    }
    fout.setf(std::ios::fixed);
    fout.precision(6);
    for (const auto &p : cloud->points) {
      fout << p.x << " " << p.y << " " << p.z << "\n";
    }
    RCLCPP_INFO(node->get_logger(),
                "[LidarAutoPick] saved %zu points to %s",
                cloud->size(), file_path.c_str());
  };

  if (img_input.empty()) {
    RCLCPP_ERROR(node->get_logger(), "No image loaded. Exiting.");
    rclcpp::shutdown();
    return 1;
  }
  if (cloud_input->empty()) {
    RCLCPP_ERROR(node->get_logger(), "No point cloud loaded. Exiting.");
    rclcpp::shutdown();
    return 1;
  }

  auto raw_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("raw_cloud", 1);
  auto colored_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("colored_cloud", 1);
  auto aligned_lidar_centers_pub =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_lidar_centers", 1);

  pcl::PointCloud<Common::Point>::Ptr near_cloud(new pcl::PointCloud<Common::Point>);
  const float max_range = 5.0f;
  for (const auto &pt : *cloud_input) {
    float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    if (dist <= max_range) near_cloud->push_back(pt);
  }
  pcl::PointCloud<Common::Point>::Ptr display_cloud(new pcl::PointCloud<Common::Point>);
  pcl::VoxelGrid<Common::Point> voxel;
  voxel.setInputCloud(near_cloud);
  voxel.setLeafSize(0.01f, 0.01f, 0.01f);
  voxel.filter(*display_cloud);
  RCLCPP_INFO(node->get_logger(), "Display cloud: %zu -> %zu (5m) -> %zu (voxel)",
              cloud_input->size(), near_cloud->size(), display_cloud->size());

  sensor_msgs::msg::PointCloud2 raw_msg;
  pcl::toROSMsg(*display_cloud, raw_msg);
  raw_msg.header.frame_id = "map";

  Eigen::Matrix4f last_transformation = Eigen::Matrix4f::Identity();
  bool has_last_transformation = false;
  const double max_rmse_accept = params.max_rmse_accept;
  const double max_translation_jump_m = params.max_translation_jump_m;
  const double max_rotation_jump_deg = params.max_rotation_jump_deg;
  int round_idx = 1;
  int accepted_rounds = 0;
  bool keep_running = true;
  bool force_manual_pick_next_round = false;

  auto detectQrSingleFrame = [&](pcl::PointCloud<pcl::PointXYZ>::Ptr centers_cloud) -> bool {
    centers_cloud->clear();
    qrDetectPtr->detect_qr(img_input, centers_cloud);
    return centers_cloud->size() == 4;
  };

  while (rclcpp::ok() && keep_running)
  {
    RCLCPP_INFO(node->get_logger(), "========== Calibration round %d ==========", round_idx);

    pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloud<PointXYZ>::Ptr qr_center_cloud(new PointCloud<PointXYZ>);
    qr_center_cloud->reserve(4);
    bool has_loaded_lidar_points = false;

    if (apriltag_mode && params.use_lidar_auto_pick && !force_manual_pick_next_round) {
      has_loaded_lidar_points = loadLidarPoints(params.lidar_points_file, clicked_lidar_centers);
      if (has_loaded_lidar_points) {
        RCLCPP_INFO(node->get_logger(),
                    "[LidarAutoPick] loaded 4 lidar points from %s. Skip manual RViz pick.",
                    params.lidar_points_file.c_str());
      } else {
        RCLCPP_INFO(node->get_logger(),
                    "[LidarAutoPick] no valid saved points found. Need manual RViz pick this round.");
      }
    }

    if (need_image_pick_first) {
      if (!detectQrSingleFrame(qr_center_cloud)) {
        RCLCPP_ERROR(node->get_logger(),
                     "Camera side did not get 4 valid target points in the selected single frame.");
        break;
      }
    }

    if (params.use_point_pick && !(apriltag_mode && has_loaded_lidar_points)) {
      const double half_w = params.delta_width_circles / 2.0 + params.circle_radius;
      const double half_h = params.delta_height_circles / 2.0 + params.circle_radius;
      const double expand = std::max(half_w, half_h) + params.pick_padding;
      std::atomic<bool> pick_done{false};

      sensor_msgs::msg::PointCloud2 preview_colored_msg;
      bool has_preview_colored = false;
      if (apriltag_mode && has_last_transformation) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr preview_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
        projectPointCloudToImage(cloud_input, last_transformation,
                                 qrDetectPtr->cameraMatrix_, qrDetectPtr->distCoeffs_,
                                 img_input, preview_colored, params.camera_model);
        if (!preview_colored->empty()) {
          pcl::toROSMsg(*preview_colored, preview_colored_msg);
          preview_colored_msg.header.frame_id = "map";
          has_preview_colored = true;
          RCLCPP_INFO(node->get_logger(),
                      "Preview colored cloud published from last round extrinsic.");
        }
      }

      auto sub = node->create_subscription<geometry_msgs::msg::PointStamped>(
          "/clicked_point", 10,
          [&](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
            if (pick_done.load()) return;
            const auto &p = msg->point;
            if (apriltag_mode) {
              pcl::PointXYZ point;
              point.x = static_cast<float>(p.x);
              point.y = static_cast<float>(p.y);
              point.z = static_cast<float>(p.z);
              clicked_lidar_centers->push_back(point);
              RCLCPP_INFO(node->get_logger(),
                          "Picked lidar point %zu/%d: (%.3f, %.3f, %.3f)",
                          clicked_lidar_centers->size(), required_picks, p.x, p.y, p.z);
              if (static_cast<int>(clicked_lidar_centers->size()) >= required_picks) {
                pick_done.store(true);
              }
            } else {
              RCLCPP_INFO(node->get_logger(),
                          "Clicked at (%.3f, %.3f, %.3f), expand=%.2f",
                          p.x, p.y, p.z, expand);
              lidarDetectPtr->setFilterBounds(
                  p.x - expand, p.x + expand,
                  p.y - expand, p.y + expand,
                  p.z - expand, p.z + expand);
              RCLCPP_INFO(node->get_logger(),
                          "Filter bounds: x=[%.2f, %.2f] y=[%.2f, %.2f] z=[%.2f, %.2f]",
                          p.x - expand, p.x + expand,
                          p.y - expand, p.y + expand,
                          p.z - expand, p.z + expand);
              pick_done.store(true);
            }
          });

      if (apriltag_mode) {
        RCLCPP_INFO(node->get_logger(),
                    "AprilTag mode: click %d points on the board in RViz2 "
                    "(use 'Publish Point' tool).", required_picks);
      } else {
        RCLCPP_INFO(node->get_logger(),
                    "Click once on the calibration board in RViz2 "
                    "(use 'Publish Point' tool). ROI will auto-expand by %.2fm.", expand);
      }

      rclcpp::WallRate pick_rate(15);
      while (rclcpp::ok() && !pick_done.load()) {
        raw_msg.header.stamp = node->now();
        raw_pub->publish(raw_msg);
        if (has_preview_colored) {
          preview_colored_msg.header.stamp = raw_msg.header.stamp;
          colored_cloud_pub->publish(preview_colored_msg);
        }
        rclcpp::spin_some(node);
        pick_rate.sleep();
      }
      sub.reset();

      if (!rclcpp::ok()) break;
    }

    if (!need_image_pick_first) {
      if (!detectQrSingleFrame(qr_center_cloud)) {
        RCLCPP_ERROR(node->get_logger(),
                     "Camera side did not get 4 valid target points in the selected single frame.");
        break;
      }
    }

    PointCloud<PointXYZ>::Ptr lidar_center_cloud(new PointCloud<PointXYZ>);
    lidar_center_cloud->reserve(4);

    if (apriltag_mode) {
      if (!params.use_point_pick || clicked_lidar_centers->size() < 4) {
        if (params.use_lidar_auto_pick && clicked_lidar_centers->size() == 4) {
          *lidar_center_cloud = *clicked_lidar_centers;
        } else {
          RCLCPP_ERROR(node->get_logger(),
                       "AprilTag mode requires either manual 4-clicks "
                       "(use_point_pick=true) or valid saved lidar points "
                       "(use_lidar_auto_pick=true + lidar_points_file with 4 points).");
          break;
        }
      } else {
        *lidar_center_cloud = *clicked_lidar_centers;
      }
    } else {
      switch (dataPreprocessPtr->lidar_type_) {
      case LiDARType::Solid:
        lidarDetectPtr->detect_solid_lidar(cloud_input, lidar_center_cloud);
        break;
      case LiDARType::Mech:
        lidarDetectPtr->detect_mech_lidar(cloud_input, lidar_center_cloud);
        break;
      default:
        std::cerr << BOLDYELLOW << "[Main] Unknown LiDAR type." << RESET << std::endl;
        break;
      }
    }

    PointCloud<PointXYZ>::Ptr qr_centers(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr lidar_centers(new PointCloud<PointXYZ>);
    if (apriltag_mode) {
      *qr_centers = *qr_center_cloud;
      *lidar_centers = *lidar_center_cloud;
    } else {
      sortPatternCenters(qr_center_cloud, qr_centers, "camera");
      sortPatternCenters(lidar_center_cloud, lidar_centers, "lidar");
    }

    Eigen::Matrix4f transformation;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
    svd.estimateRigidTransformation(*lidar_centers, *qr_centers, transformation);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
    aligned_lidar_centers->reserve(lidar_centers->size());
    alignPointCloud(lidar_centers, aligned_lidar_centers, transformation);

    double rmse = computeRMSE(qr_centers, aligned_lidar_centers);
    if (rmse > 0) {
      std::cout << BOLDYELLOW << "[Result] RMSE: " << BOLDRED << std::fixed << std::setprecision(4)
                << rmse << " m" << RESET << std::endl;
    }

    std::cout << BOLDYELLOW
              << "[Result] Single-scene calibration: extrinsic parameters T_cam_lidar = "
              << RESET << std::endl;
    std::cout << BOLDCYAN << std::fixed << std::setprecision(6) << transformation << RESET
              << std::endl;

    bool accept_round = true;
    if (rmse <= 0 || rmse > max_rmse_accept) {
      RCLCPP_WARN(node->get_logger(),
                  "Round %d rejected by RMSE gate: %.4f > %.3f (or invalid).",
                  round_idx, rmse, max_rmse_accept);
      accept_round = false;
    }

    if (accept_round && has_last_transformation) {
      Eigen::Vector3f t_last = last_transformation.block<3, 1>(0, 3);
      Eigen::Vector3f t_curr = transformation.block<3, 1>(0, 3);
      double trans_jump = (t_curr - t_last).norm();

      Eigen::Matrix3f R_last = last_transformation.block<3, 3>(0, 0);
      Eigen::Matrix3f R_curr = transformation.block<3, 3>(0, 0);
      Eigen::Matrix3f R_delta = R_last.transpose() * R_curr;
      double cos_angle = (static_cast<double>(R_delta.trace()) - 1.0) * 0.5;
      cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
      constexpr double kPi = 3.14159265358979323846;
      double rot_jump_deg = std::acos(cos_angle) * 180.0 / kPi;

      if (trans_jump > max_translation_jump_m || rot_jump_deg > max_rotation_jump_deg) {
        RCLCPP_WARN(node->get_logger(),
                    "Round %d rejected by jump gate: trans_jump=%.3fm (max %.3fm), "
                    "rot_jump=%.2fdeg (max %.2fdeg).",
                    round_idx, trans_jump, max_translation_jump_m,
                    rot_jump_deg, max_rotation_jump_deg);
        accept_round = false;
      }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    projectPointCloudToImage(cloud_input, transformation, qrDetectPtr->cameraMatrix_,
                             qrDetectPtr->distCoeffs_, img_input, colored_cloud, params.camera_model);

    if (accept_round) {
      saveTargetHoleCenters(lidar_centers, qr_centers, params);
      saveCalibrationResults(params, transformation, colored_cloud, qrDetectPtr->imageCopy_);
      if (apriltag_mode && params.use_lidar_auto_pick &&
          clicked_lidar_centers->size() >= static_cast<size_t>(required_picks)) {
        saveLidarPoints(params.lidar_points_file, clicked_lidar_centers);
      }
      last_transformation = transformation;
      has_last_transformation = true;
      force_manual_pick_next_round = false;
      ++accepted_rounds;
      RCLCPP_INFO(node->get_logger(), "Round %d accepted.", round_idx);
      if (accepted_rounds >= std::max(1, params.max_rounds)) {
        RCLCPP_INFO(node->get_logger(),
                    "Reached max_rounds=%d (accepted=%d). Exiting.",
                    params.max_rounds, accepted_rounds);
        break;
      }
    } else {
      RCLCPP_WARN(node->get_logger(),
                  "Round %d discarded. Previous accepted extrinsic is kept for preview.",
                  round_idx);
      if (apriltag_mode && params.use_lidar_auto_pick) {
        force_manual_pick_next_round = true;
        RCLCPP_WARN(node->get_logger(),
                    "[LidarAutoPick] RMSE/jump check failed. "
                    "Next round will require manual 4-click re-pick.");
      }
    }

    if (DEBUG) {
      auto stamp = node->now();

      sensor_msgs::msg::PointCloud2 qr_centers_msg;
      pcl::toROSMsg(*qr_centers, qr_centers_msg);
      qr_centers_msg.header.stamp = stamp;
      qr_centers_msg.header.frame_id = "map";
      qrDetectPtr->qr_pub_->publish(qr_centers_msg);

      sensor_msgs::msg::PointCloud2 lidar_centers_msg;
      pcl::toROSMsg(*lidar_centers, lidar_centers_msg);
      lidar_centers_msg.header = qr_centers_msg.header;
      lidarDetectPtr->center_pub_->publish(lidar_centers_msg);

      sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
      pcl::toROSMsg(*lidarDetectPtr->getFilteredCloud(), filtered_cloud_msg);
      filtered_cloud_msg.header = qr_centers_msg.header;
      lidarDetectPtr->filtered_pub_->publish(filtered_cloud_msg);

      sensor_msgs::msg::PointCloud2 plane_cloud_msg;
      pcl::toROSMsg(*lidarDetectPtr->getPlaneCloud(), plane_cloud_msg);
      plane_cloud_msg.header = qr_centers_msg.header;
      lidarDetectPtr->plane_pub_->publish(plane_cloud_msg);

      sensor_msgs::msg::PointCloud2 aligned_cloud_msg;
      pcl::toROSMsg(*lidarDetectPtr->getAlignedCloud(), aligned_cloud_msg);
      aligned_cloud_msg.header = qr_centers_msg.header;
      lidarDetectPtr->aligned_pub_->publish(aligned_cloud_msg);

      sensor_msgs::msg::PointCloud2 edge_cloud_msg;
      pcl::toROSMsg(*lidarDetectPtr->getEdgeCloud(), edge_cloud_msg);
      edge_cloud_msg.header = qr_centers_msg.header;
      lidarDetectPtr->edge_pub_->publish(edge_cloud_msg);

      sensor_msgs::msg::PointCloud2 lidar_centers_z0_msg;
      pcl::toROSMsg(*lidarDetectPtr->getCenterZ0Cloud(), lidar_centers_z0_msg);
      lidar_centers_z0_msg.header = qr_centers_msg.header;
      lidarDetectPtr->center_z0_pub_->publish(lidar_centers_z0_msg);

      sensor_msgs::msg::PointCloud2 aligned_lidar_centers_msg;
      pcl::toROSMsg(*aligned_lidar_centers, aligned_lidar_centers_msg);
      aligned_lidar_centers_msg.header = qr_centers_msg.header;
      aligned_lidar_centers_pub->publish(aligned_lidar_centers_msg);

      sensor_msgs::msg::PointCloud2 colored_cloud_msg;
      pcl::toROSMsg(*colored_cloud, colored_cloud_msg);
      colored_cloud_msg.header = qr_centers_msg.header;
      colored_cloud_pub->publish(colored_cloud_msg);
    }

    RCLCPP_INFO(node->get_logger(),
                "Round %d done. Auto-start next round. Press Ctrl+C to stop.", round_idx);
    ++round_idx;
  }

  rclcpp::shutdown();
  return 0;
}
