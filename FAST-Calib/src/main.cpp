/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "qr_detect.hpp"
#include "lidar_detect.hpp"
#include "data_preprocess.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>

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

    if (params.use_point_pick) {
        auto raw_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("raw_cloud", 1);

        pcl::PointCloud<Common::Point>::Ptr near_cloud(new pcl::PointCloud<Common::Point>);
        const float max_range = 5.0f;
        for (const auto& pt : *cloud_input) {
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

        const double half_w = params.delta_width_circles / 2.0 + params.circle_radius;
        const double half_h = params.delta_height_circles / 2.0 + params.circle_radius;
        const double expand = std::max(half_w, half_h) + params.pick_padding;
        std::atomic<bool> pick_done{false};

        auto sub = node->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10,
            [&](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
                if (pick_done.load()) return;
                const auto& p = msg->point;
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
            });

        RCLCPP_INFO(node->get_logger(),
            "Click once on the calibration board in RViz2 "
            "(use 'Publish Point' tool). ROI will auto-expand by %.2fm.", expand);

        rclcpp::WallRate pick_rate(1);
        while (rclcpp::ok() && !pick_done.load()) {
            raw_msg.header.stamp = node->now();
            raw_pub->publish(raw_msg);
            rclcpp::spin_some(node);
            pick_rate.sleep();
        }

        sub.reset();

        if (!rclcpp::ok()) {
            rclcpp::shutdown();
            return 0;
        }
    }

    PointCloud<PointXYZ>::Ptr qr_center_cloud(new PointCloud<PointXYZ>);
    qr_center_cloud->reserve(4);
    qrDetectPtr->detect_qr(img_input, qr_center_cloud);

    PointCloud<PointXYZ>::Ptr lidar_center_cloud(new PointCloud<PointXYZ>);
    lidar_center_cloud->reserve(4);
    
    switch (dataPreprocessPtr->lidar_type_)
    {
        case LiDARType::Solid:
            lidarDetectPtr->detect_solid_lidar(cloud_input, lidar_center_cloud);
            break;

        case LiDARType::Mech:
            lidarDetectPtr->detect_mech_lidar(cloud_input, lidar_center_cloud);
            break;

        default:
            std::cerr << BOLDYELLOW 
                    << "[Main] Unknown LiDAR type." 
                    << RESET << std::endl;
            break;
    }

    PointCloud<PointXYZ>::Ptr qr_centers(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr lidar_centers(new PointCloud<PointXYZ>);
    sortPatternCenters(qr_center_cloud, qr_centers, "camera");
    sortPatternCenters(lidar_center_cloud, lidar_centers, "lidar");

    saveTargetHoleCenters(lidar_centers, qr_centers, params);

    Eigen::Matrix4f transformation;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
    svd.estimateRigidTransformation(*lidar_centers, *qr_centers, transformation);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
    aligned_lidar_centers->reserve(lidar_centers->size());
    alignPointCloud(lidar_centers, aligned_lidar_centers, transformation);
    
    double rmse = computeRMSE(qr_centers, aligned_lidar_centers);
    if (rmse > 0) 
    {
      std::cout << BOLDYELLOW << "[Result] RMSE: " << BOLDRED << std::fixed << std::setprecision(4)
      << rmse << " m" << RESET << std::endl;
    }

    std::cout << BOLDYELLOW << "[Result] Single-scene calibration: extrinsic parameters T_cam_lidar = " << RESET << std::endl;
    std::cout << BOLDCYAN << std::fixed << std::setprecision(6) << transformation << RESET << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    projectPointCloudToImage(cloud_input, transformation, qrDetectPtr->cameraMatrix_, qrDetectPtr->distCoeffs_, img_input, colored_cloud);

    saveCalibrationResults(params, transformation, colored_cloud, qrDetectPtr->imageCopy_);

    auto colored_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("colored_cloud", 1);
    auto aligned_lidar_centers_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_lidar_centers", 1);

    rclcpp::WallRate rate(1);
    while (rclcpp::ok()) 
    {
      if (DEBUG) 
      {
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
      rclcpp::spin_some(node);
      rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
