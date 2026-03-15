/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef DATA_PREPROCESS_HPP
#define DATA_PREPROCESS_HPP

#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <filesystem>
#include "common_lib.h"

using namespace std;

enum class LiDARType : int {
    Unknown = 0,
    Solid   = 1,
    Mech    = 2
};

class DataPreprocess
{
public:
    pcl::PointCloud<Common::Point>::Ptr cloud_input_;
    cv::Mat img_input_;
    LiDARType lidar_type_{LiDARType::Unknown};
    LiDARType lidarType() const { return lidar_type_; }

    DataPreprocess(Params &params, rclcpp::Logger logger = rclcpp::get_logger("data_preprocess"))
        : cloud_input_(new pcl::PointCloud<Common::Point>),
          logger_(logger)
    {
        string bag_path    = params.bag_path;
        string image_path  = params.image_path;
        string lidar_topic = params.lidar_topic;
        string image_topic = params.image_topic;

        // Open ROS 2 bag
        if (bag_path.empty()) {
            RCLCPP_ERROR(logger_, "bag_path is empty");
            return;
        }

        if (!std::filesystem::exists(bag_path)) {
            RCLCPP_ERROR(logger_, "Bag path does not exist: %s", bag_path.c_str());
            return;
        }

        RCLCPP_INFO(logger_, "Loading rosbag: %s", bag_path.c_str());

        rosbag2_cpp::Reader reader;
        try {
            reader.open(bag_path);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(logger_, "Failed to open bag: %s", e.what());
            return;
        }

        // Deserializers
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_serializer;
        rclcpp::Serialization<sensor_msgs::msg::Image> img_serializer;
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage> cimg_serializer;

        bool got_image = false;

        while (reader.has_next())
        {
            auto msg = reader.read_next();
            const std::string& topic = msg->topic_name;

            // PointCloud2
            if (topic == lidar_topic)
            {
                auto pcl_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                rclcpp::SerializedMessage serialized(*msg->serialized_data);
                pc2_serializer.deserialize_message(&serialized, pcl_msg.get());

                bool has_ring = false;
                for (const auto &f : pcl_msg->fields) {
                    if (f.name == "ring") { has_ring = true; break; }
                }

                lidar_type_ = has_ring ? LiDARType::Mech : LiDARType::Solid;

                // Read points using field iterators
                sensor_msgs::PointCloud2ConstIterator<float> it_x(*pcl_msg, "x");
                sensor_msgs::PointCloud2ConstIterator<float> it_y(*pcl_msg, "y");
                sensor_msgs::PointCloud2ConstIterator<float> it_z(*pcl_msg, "z");

                std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<std::uint16_t>> it_ring_ptr;
                if (has_ring) {
                    it_ring_ptr.reset(new sensor_msgs::PointCloud2ConstIterator<std::uint16_t>(*pcl_msg, "ring"));
                }

                const size_t n = static_cast<size_t>(pcl_msg->width) * pcl_msg->height;
                cloud_input_->reserve(cloud_input_->size() + n);

                for (size_t i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z)
                {
                    Common::Point p;
                    p.x = *it_x;
                    p.y = *it_y;
                    p.z = *it_z;

                    if (has_ring) {
                        p.ring = **it_ring_ptr;
                        ++(*it_ring_ptr);
                    } else {
                        p.ring = 0xFFFF;
                    }

                    cloud_input_->push_back(p);
                }
                continue;
            }

            // Image (uncompressed)
            if (!got_image && topic == image_topic)
            {
                auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
                rclcpp::SerializedMessage serialized(*msg->serialized_data);
                img_serializer.deserialize_message(&serialized, img_msg.get());

                try {
                    auto cv_ptr = cv_bridge::toCvCopy(*img_msg, "bgr8");
                    img_input_ = cv_ptr->image;
                    got_image = true;
                    RCLCPP_INFO(logger_, "Loaded image from bag topic: %s (%dx%d)",
                                image_topic.c_str(), img_input_.cols, img_input_.rows);
                } catch (const cv_bridge::Exception &e) {
                    RCLCPP_WARN(logger_, "cv_bridge error on topic %s: %s", image_topic.c_str(), e.what());
                }
                continue;
            }

            // CompressedImage
            if (!got_image && (topic == image_topic + "/compressed" || topic == image_topic))
            {
                try {
                    auto cimg_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
                    rclcpp::SerializedMessage serialized(*msg->serialized_data);
                    cimg_serializer.deserialize_message(&serialized, cimg_msg.get());

                    img_input_ = cv::imdecode(cv::Mat(cimg_msg->data), cv::IMREAD_COLOR);
                    if (!img_input_.empty()) {
                        got_image = true;
                        RCLCPP_INFO(logger_, "Loaded compressed image from bag (%dx%d)",
                                    img_input_.cols, img_input_.rows);
                    }
                } catch (...) {
                    // Not a CompressedImage, skip
                }
                continue;
            }
        }

        RCLCPP_INFO(logger_, "Loaded %zu points from the rosbag.", cloud_input_->size());

        // Fallback: load image from file if not found in bag
        if (!got_image) {
            if (!image_path.empty()) {
                img_input_ = cv::imread(image_path, cv::IMREAD_UNCHANGED);
                if (img_input_.empty()) {
                    RCLCPP_ERROR(logger_, "Failed to load image from file: %s", image_path.c_str());
                } else {
                    RCLCPP_INFO(logger_, "Loaded image from file: %s (%dx%d)",
                                image_path.c_str(), img_input_.cols, img_input_.rows);
                }
            } else {
                RCLCPP_WARN(logger_, "No image found in bag (topic: %s) and no image_path specified",
                            image_topic.c_str());
            }
        }
    }

private:
    rclcpp::Logger logger_;
};

typedef std::shared_ptr<DataPreprocess> DataPreprocessPtr;

#endif // DATA_PREPROCESS_HPP
