/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef QR_DETECT_HPP
#define QR_DETECT_HPP
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <opencv2/aruco.hpp>
#include <limits>
#include <set>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include "common_lib.h"

class QRDetect 
{
  private:
    double marker_size_, delta_width_qr_center_, delta_height_qr_center_;
    double delta_width_circles_, delta_height_circles_;
    int min_detected_markers_;
    bool use_image_pick_;
    bool use_vlm_auto_pick_;
    std::string vlm_points_file_;
    std::string target_mode_;
    std::string camera_model_;
    std::vector<int> board_ids_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    rclcpp::Logger logger_;

    int dictionaryIdFromName(const std::string& name)
    {
      if (name == "aruco_6x6_250") return cv::aruco::DICT_6X6_250;
      if (name == "apriltag_16h5") return cv::aruco::DICT_APRILTAG_16h5;
      if (name == "apriltag_25h9") return cv::aruco::DICT_APRILTAG_25h9;
      if (name == "apriltag_36h10") return cv::aruco::DICT_APRILTAG_36h10;
      if (name == "apriltag_36h11") return cv::aruco::DICT_APRILTAG_36h11;
      RCLCPP_WARN(logger_, "Unknown tag_dictionary '%s', fallback to aruco_6x6_250", name.c_str());
      return cv::aruco::DICT_6X6_250;
    }

 
  public:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr qr_pub_;
    cv::Mat imageCopy_;
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;

    QRDetect(rclcpp::Node::SharedPtr node, Params& params)
      : logger_(node->get_logger())
    {
      marker_size_ = params.marker_size;
      delta_width_qr_center_ = params.delta_width_qr_center;
      delta_height_qr_center_ = params.delta_height_qr_center;
      delta_width_circles_ = params.delta_width_circles;
      delta_height_circles_ = params.delta_height_circles;
      min_detected_markers_ = params.min_detected_markers;
      use_image_pick_ = params.use_image_pick;
      use_vlm_auto_pick_ = params.use_vlm_auto_pick;
      vlm_points_file_ = params.vlm_points_file;
      target_mode_ = params.target_mode;
      camera_model_ = params.camera_model;
      std::transform(camera_model_.begin(), camera_model_.end(), camera_model_.begin(),
                     [](unsigned char c){ return static_cast<char>(std::tolower(c)); });

      board_ids_.clear();
      for (auto id : params.board_ids) {
        board_ids_.push_back(static_cast<int>(id));
      }
      if (board_ids_.size() != 4) {
        RCLCPP_WARN(logger_, "board_ids size=%zu, expected 4. Fallback to {1,2,4,3}.", board_ids_.size());
        board_ids_ = {1, 2, 4, 3};
      }
      RCLCPP_INFO(logger_, "Using board_ids: [%d, %d, %d, %d]",
                  board_ids_[0], board_ids_[1], board_ids_[2], board_ids_[3]);
      
      cameraMatrix_ = (cv::Mat_<double>(3, 3) << params.fx, 0, params.cx,
                                                 0, params.fy, params.cy,
                                                 0,         0,        1);

      // For fisheye model, map yaml {k1,k2,p1,p2} -> {k1,k2,k3,k4}.
      if (camera_model_ == "fisheye") {
        distCoeffs_ = (cv::Mat_<double>(4, 1) << params.k1, params.k2, params.p1, params.p2);
      } else {
        distCoeffs_ = (cv::Mat_<double>(1, 5) << params.k1, params.k2, params.p1, params.p2, 0);
      }

      dictionary_ = cv::aruco::getPredefinedDictionary(dictionaryIdFromName(params.tag_dictionary));
      RCLCPP_INFO(logger_, "Target mode: %s, tag_dictionary: %s",
                  target_mode_.c_str(), params.tag_dictionary.c_str());
      RCLCPP_INFO(logger_, "camera_model: %s", camera_model_.c_str());
      RCLCPP_INFO(logger_, "use_image_pick: %s", use_image_pick_ ? "true" : "false");
      RCLCPP_INFO(logger_, "use_vlm_auto_pick: %s, vlm_points_file: %s",
                  use_vlm_auto_pick_ ? "true" : "false", vlm_points_file_.c_str());

      qr_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("qr_cloud", 1);
    }

    Point2f projectPointDist(cv::Point3f pt_cv, const Mat intrinsics, const Mat distCoeffs) 
    {
      vector<Point3f> input{pt_cv};
      vector<Point2f> projectedPoints;
      projectedPoints.resize(1);
      if (camera_model_ == "fisheye" && distCoeffs.total() == 4) {
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::fisheye::projectPoints(input, projectedPoints, rvec, tvec, intrinsics, distCoeffs);
      } else {
        projectPoints(input, Mat::zeros(3, 1, CV_64FC1), Mat::zeros(3, 1, CV_64FC1),
                      intrinsics, distCoeffs, projectedPoints);
      }
      return projectedPoints[0];
    }

    struct ImageClickState {
      cv::Mat shown;
      std::vector<cv::Point2f> clicks;
      int required = 4;
      bool done = false;
    };

    static void onMouse(int event, int x, int y, int, void* userdata) {
      if (event != cv::EVENT_LBUTTONDOWN || userdata == nullptr) return;
      auto* state = reinterpret_cast<ImageClickState*>(userdata);
      if (state->done) return;
      state->clicks.emplace_back(static_cast<float>(x), static_cast<float>(y));
      cv::circle(state->shown, cv::Point(x, y), 6, cv::Scalar(0, 255, 255), 2);
      cv::putText(state->shown, std::to_string(state->clicks.size()),
                  cv::Point(x + 8, y - 8), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                  cv::Scalar(0, 255, 255), 2);
      if (static_cast<int>(state->clicks.size()) >= state->required) {
        state->done = true;
      }
    }

    bool loadVlmPoints(std::vector<cv::Point2f>& points)
    {
      points.clear();
      if (vlm_points_file_.empty()) return false;
      std::ifstream fin(vlm_points_file_);
      if (!fin.is_open()) {
        RCLCPP_WARN(logger_, "[VLM] Cannot open vlm_points_file: %s", vlm_points_file_.c_str());
        return false;
      }
      std::string line;
      while (std::getline(fin, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        float u = 0.0f, v = 0.0f;
        if (!(iss >> u >> v)) continue;
        points.emplace_back(u, v);
      }
      if (points.size() != TARGET_NUM_CIRCLES) {
        RCLCPP_WARN(logger_, "[VLM] Expect %d points, got %zu from %s",
                    TARGET_NUM_CIRCLES, points.size(), vlm_points_file_.c_str());
        points.clear();
        return false;
      }
      return true;
    }

    void comb(int N, int K, std::vector<std::vector<int>> &groups) {
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

    void detect_qr(cv::Mat &image, pcl::PointCloud<pcl::PointXYZ>::Ptr centers_cloud) 
    {      
      cv::Mat image_proc;
      image.copyTo(image_proc);
      cv::Mat poseDist = distCoeffs_.clone();
      if (camera_model_ == "fisheye") {
        // ArUco pose APIs use pinhole distortion model.
        // For fisheye cameras, undistort first and run pose with zero pinhole distortion.
        cv::fisheye::undistortImage(image, image_proc, cameraMatrix_, distCoeffs_, cameraMatrix_);
        poseDist = cv::Mat::zeros(1, 5, CV_64FC1);
      }
      image_proc.copyTo(imageCopy_);

      std::vector<std::vector<cv::Point3f>> boardCorners;
      std::vector<cv::Point3f> boardCircleCenters;
      float width = delta_width_qr_center_;
      float height = delta_height_qr_center_;
      float circle_width = delta_width_circles_ / 2.;
      float circle_height = delta_height_circles_ / 2.;
      boardCorners.resize(4);
      for (int i = 0; i < 4; ++i) {
        int x_qr_center =
            (i % 3) == 0 ? -1 : 1;
        int y_qr_center =
            (i < 2) ? 1 : -1;
        float x_center = x_qr_center * width;
        float y_center = y_qr_center * height;

        cv::Point3f circleCenter3d(x_qr_center * circle_width,
                                  y_qr_center * circle_height, 0);
        boardCircleCenters.push_back(circleCenter3d);
        for (int j = 0; j < 4; ++j) {
          int x_qr = (j % 3) == 0 ? -1 : 1;
          int y_qr = (j < 2) ? 1 : -1;
          cv::Point3f pt3d(x_center + x_qr * marker_size_ / 2.,
                          y_center + y_qr * marker_size_ / 2., 0);
          boardCorners[i].push_back(pt3d);
        }
      }

      std::vector<int> boardIds = board_ids_;
      cv::Ptr<cv::aruco::Board> board =
          cv::aruco::Board::create(boardCorners, dictionary_, boardIds);

      cv::Ptr<cv::aruco::DetectorParameters> parameters =
          cv::aruco::DetectorParameters::create();

    #if (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION <= 2) || CV_MAJOR_VERSION < 3
      parameters->doCornerRefinement = true;
    #else
      parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    #endif

      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(image_proc, dictionary_, corners, ids, parameters);

      if (target_mode_ == "apriltag_grid" && ids.empty()) {
        std::vector<std::pair<std::string, int>> dict_candidates = {
          {"apriltag_36h11", cv::aruco::DICT_APRILTAG_36h11},
          {"apriltag_36h10", cv::aruco::DICT_APRILTAG_36h10},
          {"apriltag_25h9",  cv::aruco::DICT_APRILTAG_25h9},
          {"apriltag_16h5",  cv::aruco::DICT_APRILTAG_16h5},
          {"aruco_6x6_250",  cv::aruco::DICT_6X6_250}
        };

        size_t best_count = 0;
        std::string best_name = "none";
        std::vector<int> best_ids;
        std::vector<std::vector<cv::Point2f>> best_corners;
        for (const auto& cand : dict_candidates) {
          auto dict = cv::aruco::getPredefinedDictionary(cand.second);
          std::vector<int> test_ids;
          std::vector<std::vector<cv::Point2f>> test_corners;
          cv::aruco::detectMarkers(image_proc, dict, test_corners, test_ids, parameters);
          RCLCPP_INFO(logger_, "[AprilTag] dictionary=%s detected=%zu marker(s)",
                      cand.first.c_str(), test_ids.size());
          if (test_ids.size() > best_count) {
            best_count = test_ids.size();
            best_name = cand.first;
            best_ids = test_ids;
            best_corners = test_corners;
          }
        }
        if (best_count > 0) {
          ids = best_ids;
          corners = best_corners;
          RCLCPP_WARN(logger_,
              "[AprilTag] Switched to dictionary=%s with %zu marker(s).",
              best_name.c_str(), best_count);
        }
      }

      if (ids.size() > 0) cv::aruco::drawDetectedMarkers(imageCopy_, corners, ids);
      if (target_mode_ == "apriltag_grid" && use_image_pick_) {
        // Manual image picking path for a single selected frame.
        // This path works even when fewer than 4 tags are auto-detected.
        ImageClickState state;
        state.shown = imageCopy_.clone();
        state.required = TARGET_NUM_CIRCLES;

        const std::string win = "FAST-Calib Tag Pick";
        cv::namedWindow(win, cv::WINDOW_NORMAL);
        cv::setMouseCallback(win, onMouse, &state);
        RCLCPP_INFO(logger_, "[AprilTag] Manual image pick: click %d points in image window.",
                    TARGET_NUM_CIRCLES);
        RCLCPP_INFO(logger_, "[AprilTag] Click order must match RViz clicked order.");
        RCLCPP_INFO(logger_, "[AprilTag] This run uses only the current frame (no frame fallback).");

        while (!state.done) {
          cv::imshow(win, state.shown);
          int key = cv::waitKey(20);
          if (key == 27) {
            cv::destroyWindow(win);
            RCLCPP_ERROR(logger_, "[AprilTag] Image pick canceled by ESC.");
            centers_cloud->clear();
            return;
          }
        }
        cv::destroyWindow(win);

        if (state.clicks.size() != TARGET_NUM_CIRCLES) {
          RCLCPP_ERROR(logger_, "[AprilTag] Manual clicks=%zu, expected=%d.",
                       state.clicks.size(), TARGET_NUM_CIRCLES);
          centers_cloud->clear();
          return;
        }

        cv::Vec3d rvec(0, 0, 0), tvec(0, 0, 0);
        bool pnp_ok = cv::solvePnP(boardCircleCenters, state.clicks, cameraMatrix_, poseDist,
                                   rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        if (!pnp_ok) {
          RCLCPP_ERROR(logger_, "[AprilTag] solvePnP failed with manual image points.");
          centers_cloud->clear();
          return;
        }

        cv::aruco::drawAxis(imageCopy_, cameraMatrix_, poseDist, rvec, tvec, 0.2);

        cv::Mat R;
        cv::Rodrigues(rvec, R);
        cv::Mat t = cv::Mat::zeros(3, 1, CV_64F);
        t.at<double>(0) = tvec[0];
        t.at<double>(1) = tvec[1];
        t.at<double>(2) = tvec[2];

        centers_cloud->clear();
        for (size_t i = 0; i < boardCircleCenters.size(); ++i) {
          cv::Mat p = (cv::Mat_<double>(3, 1) << boardCircleCenters[i].x,
                                                boardCircleCenters[i].y,
                                                boardCircleCenters[i].z);
          cv::Mat pc = R * p + t;

          pcl::PointXYZ center;
          center.x = static_cast<float>(pc.at<double>(0));
          center.y = static_cast<float>(pc.at<double>(1));
          center.z = static_cast<float>(pc.at<double>(2));
          centers_cloud->push_back(center);

          cv::Point3f center3d(center.x, center.y, center.z);
          cv::Point2f uv = projectPointDist(center3d, cameraMatrix_, poseDist);
          circle(imageCopy_, uv, 5, Scalar(0, 255, 0), -1);
        }

        RCLCPP_INFO(logger_,
                    "[AprilTag] Manual image picks accepted (auto-detected markers in this frame: %zu).",
                    ids.size());
        return;
      }
      if (target_mode_ == "apriltag_grid" && ids.size() < TARGET_NUM_CIRCLES) {
        RCLCPP_WARN(logger_, "[AprilTag] detected=%zu, need at least %d in this selected frame.",
                    ids.size(), TARGET_NUM_CIRCLES);
        centers_cloud->clear();
        return;
      }

      cv::Vec3d rvec(0, 0, 0), tvec(0, 0, 0);

      const size_t required_markers =
          (target_mode_ == "apriltag_grid")
              ? static_cast<size_t>(TARGET_NUM_CIRCLES)
              : static_cast<size_t>(min_detected_markers_);

      if (ids.size() >= required_markers) 
      {
        vector<Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, cameraMatrix_,
                                            poseDist, rvecs, tvecs);

        for (size_t i = 0; i < ids.size(); i++) {
          cv::aruco::drawAxis(imageCopy_, cameraMatrix_, poseDist, rvecs[i],
                              tvecs[i], 0.1);
        }

        if (target_mode_ == "apriltag_grid") {
          std::unordered_map<int, pcl::PointXYZ> id_to_center;
          std::unordered_map<int, cv::Point2f> id_to_uv_center;
          for (size_t i = 0; i < ids.size(); ++i) {
            pcl::PointXYZ p;
            p.x = static_cast<float>(tvecs[i][0]);
            p.y = static_cast<float>(tvecs[i][1]);
            p.z = static_cast<float>(tvecs[i][2]);
            id_to_center[ids[i]] = p;
            const auto& c = corners[i];
            cv::Point2f uv_center(0.0f, 0.0f);
            for (const auto& pt : c) uv_center += pt;
            uv_center *= (1.0f / static_cast<float>(c.size()));
            id_to_uv_center[ids[i]] = uv_center;
            cv::putText(imageCopy_, std::to_string(ids[i]), uv_center + cv::Point2f(8.0f, -8.0f),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
          }

          std::vector<int> selected_ids;
          if (use_vlm_auto_pick_) {
            std::vector<cv::Point2f> vlm_clicks;
            if (!loadVlmPoints(vlm_clicks)) {
              RCLCPP_ERROR(logger_, "[VLM] Failed to load auto-pick points.");
              centers_cloud->clear();
              return;
            }
            std::set<int> used_ids;
            for (const auto& click : vlm_clicks) {
              float best_d2 = std::numeric_limits<float>::max();
              int best_id = -1;
              for (const auto& kv : id_to_uv_center) {
                if (used_ids.count(kv.first) > 0) continue;
                const auto du = kv.second.x - click.x;
                const auto dv = kv.second.y - click.y;
                const float d2 = du * du + dv * dv;
                if (d2 < best_d2) {
                  best_d2 = d2;
                  best_id = kv.first;
                }
              }
              if (best_id != -1) {
                selected_ids.push_back(best_id);
                used_ids.insert(best_id);
              }
            }
            RCLCPP_INFO(logger_, "[VLM] Auto-picked 4 image points from file.");
          } else if (use_image_pick_) {
            ImageClickState state;
            state.shown = imageCopy_.clone();
            state.required = TARGET_NUM_CIRCLES;

            const std::string win = "FAST-Calib Tag Pick";
            cv::namedWindow(win, cv::WINDOW_NORMAL);
            cv::setMouseCallback(win, onMouse, &state);
            RCLCPP_INFO(logger_, "[AprilTag] Image pick enabled: click %d tags in image window.",
                        TARGET_NUM_CIRCLES);
            RCLCPP_INFO(logger_, "[AprilTag] Click order must match RViz clicked order.");

            while (!state.done) {
              cv::imshow(win, state.shown);
              int key = cv::waitKey(20);
              if (key == 27) {
                cv::destroyWindow(win);
                RCLCPP_ERROR(logger_, "[AprilTag] Image pick canceled by ESC.");
                centers_cloud->clear();
                return;
              }
            }
            cv::destroyWindow(win);

            std::set<int> used_ids;
            for (const auto& click : state.clicks) {
              float best_d2 = std::numeric_limits<float>::max();
              int best_id = -1;
              for (const auto& kv : id_to_uv_center) {
                if (used_ids.count(kv.first) > 0) continue;
                const auto du = kv.second.x - click.x;
                const auto dv = kv.second.y - click.y;
                const float d2 = du * du + dv * dv;
                if (d2 < best_d2) {
                  best_d2 = d2;
                  best_id = kv.first;
                }
              }
              if (best_id != -1) {
                selected_ids.push_back(best_id);
                used_ids.insert(best_id);
              }
            }
          } else {
            selected_ids = board_ids_;
          }

          for (int id : selected_ids) {
            auto it = id_to_center.find(id);
            if (it == id_to_center.end()) {
              RCLCPP_WARN(logger_, "[AprilTag] tag id=%d not found in this frame.", id);
              continue;
            }
            centers_cloud->push_back(it->second);
            cv::Point3f center3d(it->second.x, it->second.y, it->second.z);
            cv::Point2f uv = projectPointDist(center3d, cameraMatrix_, poseDist);
            circle(imageCopy_, uv, 5, Scalar(0, 255, 0), -1);
          }

          if (centers_cloud->size() != TARGET_NUM_CIRCLES) {
            RCLCPP_WARN(logger_, "[AprilTag] Got %zu tag centers, expected %d.",
                        centers_cloud->size(), TARGET_NUM_CIRCLES);
            centers_cloud->clear();
            return;
          }
          if (selected_ids.size() == TARGET_NUM_CIRCLES) {
            RCLCPP_INFO(logger_, "[AprilTag] Selected tag IDs: [%d, %d, %d, %d]",
                        selected_ids[0], selected_ids[1], selected_ids[2], selected_ids[3]);
          } else {
            RCLCPP_INFO(logger_, "[AprilTag] Selected tag IDs count=%zu", selected_ids.size());
          }
        } else {
          Vec3f rvec_sin, rvec_cos;
          for (size_t i = 0; i < ids.size(); i++) {
            tvec[0] += tvecs[i][0];
            tvec[1] += tvecs[i][1];
            tvec[2] += tvecs[i][2];
            rvec_sin[0] += sin(rvecs[i][0]);
            rvec_sin[1] += sin(rvecs[i][1]);
            rvec_sin[2] += sin(rvecs[i][2]);
            rvec_cos[0] += cos(rvecs[i][0]);
            rvec_cos[1] += cos(rvecs[i][1]);
            rvec_cos[2] += cos(rvecs[i][2]);
          }

          tvec = tvec / int(ids.size());
          rvec_sin = rvec_sin / int(ids.size());
          rvec_cos = rvec_cos / int(ids.size());
          rvec[0] = atan2(rvec_sin[0], rvec_cos[0]);
          rvec[1] = atan2(rvec_sin[1], rvec_cos[1]);
          rvec[2] = atan2(rvec_sin[2], rvec_cos[2]);

          pcl::PointCloud<pcl::PointXYZ>::Ptr candidates_cloud(new pcl::PointCloud<pcl::PointXYZ>);

      #if (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION <= 2) || CV_MAJOR_VERSION < 3
          int valid = cv::aruco::estimatePoseBoard(corners, ids, board, cameraMatrix_,
                                                  poseDist, rvec, tvec);
      #else
          int valid = cv::aruco::estimatePoseBoard(corners, ids, board, cameraMatrix_,
                                                  poseDist, rvec, tvec, true);
      #endif
          (void)valid;

          cv::aruco::drawAxis(imageCopy_, cameraMatrix_, poseDist, rvec, tvec, 0.2);

          cv::Mat R(3, 3, cv::DataType<float>::type);
          cv::Rodrigues(rvec, R);

          cv::Mat t = cv::Mat::zeros(3, 1, CV_32F);
          t.at<float>(0) = tvec[0];
          t.at<float>(1) = tvec[1];
          t.at<float>(2) = tvec[2];

          cv::Mat board_transform = cv::Mat::eye(3, 4, CV_32F);
          R.copyTo(board_transform.rowRange(0, 3).colRange(0, 3));
          t.copyTo(board_transform.rowRange(0, 3).col(3));

          for (size_t i = 0; i < boardCircleCenters.size(); ++i) {
            cv::Mat mat = cv::Mat::zeros(4, 1, CV_32F);
            mat.at<float>(0, 0) = boardCircleCenters[i].x;
            mat.at<float>(1, 0) = boardCircleCenters[i].y;
            mat.at<float>(2, 0) = boardCircleCenters[i].z;
            mat.at<float>(3, 0) = 1.0;

            cv::Mat mat_qr = board_transform * mat;
            cv::Point3f center3d;
            center3d.x = mat_qr.at<float>(0, 0);
            center3d.y = mat_qr.at<float>(1, 0);
            center3d.z = mat_qr.at<float>(2, 0);

            cv::Point2f uv;
            uv = projectPointDist(center3d, cameraMatrix_, poseDist);
            circle(imageCopy_, uv, 5, Scalar(0, 255, 0), -1);

            pcl::PointXYZ qr_center;
            qr_center.x = center3d.x;
            qr_center.y = center3d.y;
            qr_center.z = center3d.z;
            candidates_cloud->push_back(qr_center);
          }

          std::vector<std::vector<int>> groups;
          comb(candidates_cloud->size(), TARGET_NUM_CIRCLES, groups);
          std::vector<double> groups_scores(groups.size());

          for (size_t i = 0; i < groups.size(); ++i) 
          {
            std::vector<pcl::PointXYZ> candidates;
            for (size_t j = 0; j < groups[i].size(); ++j) {
              pcl::PointXYZ center;
              center.x = candidates_cloud->at(groups[i][j]).x;
              center.y = candidates_cloud->at(groups[i][j]).y;
              center.z = candidates_cloud->at(groups[i][j]).z;
              candidates.push_back(center);
            }

            Square square_candidate(candidates, delta_width_circles_,
                                    delta_height_circles_);
            groups_scores[i] = square_candidate.is_valid()
                                  ? 1.0
                                  : -1;
          }

          int best_candidate_idx = -1;
          double best_candidate_score = -1;
          for (size_t i = 0; i < groups.size(); ++i) 
          {
            if (best_candidate_score == 1 && groups_scores[i] == 1) {
              RCLCPP_ERROR(logger_,
                  "[Mono] More than one set of candidates fit target's geometry. "
                  "Please, make sure your parameters are well set. Exiting callback");
              return;
            }
            if (groups_scores[i] > best_candidate_score) {
              best_candidate_score = groups_scores[i];
              best_candidate_idx = i;
            }
          }

          if (best_candidate_idx == -1) 
          {
            RCLCPP_WARN(logger_,
                "[Mono] Unable to find a candidate set that matches target's "
                "geometry");
            return;
          }

          for (size_t j = 0; j < groups[best_candidate_idx].size(); ++j) 
          {
            centers_cloud->push_back(candidates_cloud->at(groups[best_candidate_idx][j]));
          }
        }

        if (DEBUG) 
        {
          for (size_t i = 0; i < centers_cloud->size(); i++) {
            cv::Point3f pt_circle1(centers_cloud->at(i).x, centers_cloud->at(i).y,centers_cloud->at(i).z);
            cv::Point2f uv_circle1;
            uv_circle1 = projectPointDist(pt_circle1, cameraMatrix_, poseDist);
            circle(imageCopy_, uv_circle1, 2, Scalar(255, 0, 255), -1);
          }
        }
      } 
      else 
      {
        RCLCPP_WARN(logger_, "%lu marker(s) found, %zu expected. Skipping frame...", ids.size(),
                required_markers);
      }
    }
};
typedef std::shared_ptr<QRDetect> QRDetectPtr;

#endif




