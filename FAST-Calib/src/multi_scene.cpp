/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <regex>
#include <vector>
#include <string>
#include <iomanip>
#include <cmath>
#include "common_lib.h"

struct RigidResult 
{
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  double rms = 0.0;
  bool ok = false;
};
struct Block {
  std::string time_line;
  std::vector<Eigen::Vector3d> lidar_pts;
  std::vector<Eigen::Vector3d> qr_pts;
};

RigidResult SolveRigidTransformWeighted(
    const std::vector<Eigen::Vector3d>& lidar_pts,
    const std::vector<Eigen::Vector3d>& cam_pts,
    const std::vector<double>* weights = nullptr)
{
    RigidResult out; out.ok = false;
    const size_t N = lidar_pts.size();
    if (N < 3 || cam_pts.size() != N) return out;

    std::vector<double> w(N, 1.0);
    if (weights && weights->size() == N) w = *weights;
    double wsum = 0.0;
    for (double wi : w) wsum += wi;
    if (wsum <= 0) return out;

    Eigen::Vector3d muL = Eigen::Vector3d::Zero();
    Eigen::Vector3d muC = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < N; ++i) 
    {
        muL += w[i] * lidar_pts[i];
        muC += w[i] * cam_pts[i];
    }
    muL /= wsum; muC /= wsum;

    Eigen::Matrix3d Sigma = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < N; ++i) 
    {
        Eigen::Vector3d l = lidar_pts[i] - muL;
        Eigen::Vector3d c = cam_pts[i] - muC;
        Sigma += w[i] * (l * c.transpose());
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(Sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d R = V * U.transpose();
    if (R.determinant() < 0) 
    {
        Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
        D(2,2) = -1;
        R = V * D * U.transpose();
    }
    Eigen::Vector3d t = muC - R * muL;

    double rss = 0.0;
    for (size_t i = 0; i < N; ++i) 
    {
        Eigen::Vector3d r = (R * lidar_pts[i] + t) - cam_pts[i];
        rss += w[i] * r.squaredNorm();
    }
    out.R = R; out.t = t; out.rms = std::sqrt(rss / wsum); out.ok = true;
    return out;
}

static bool parseCentersLine(const std::string& line, std::vector<Eigen::Vector3d>& out_pts)
{
    std::regex brace_re("\\{([^\\}]*)\\}");
    auto begin = std::sregex_iterator(line.begin(), line.end(), brace_re);
    auto end   = std::sregex_iterator();

    out_pts.clear();
    for (auto it = begin; it != end; ++it) {
        std::string xyz = (*it)[1];
        xyz.erase(remove_if(xyz.begin(), xyz.end(), ::isspace), xyz.end());
        std::vector<double> vals;
        std::stringstream ss(xyz);
        std::string tok;
        while (std::getline(ss, tok, ',')) {
        try {
            vals.push_back(std::stod(tok));
        } catch (...) { return false; }
        }
        if (vals.size() != 3) return false;
        out_pts.emplace_back(vals[0], vals[1], vals[2]);
    }
    return !out_pts.empty();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("multi_fast_calib");
    auto logger = node->get_logger();

    Params params = loadParameters(node);

    if (params.output_path.back() != '/') params.output_path += '/';
    std::string midtxt_path = params.output_path + "circle_center_record.txt";
    std::string multi_output_path = params.output_path + "multi_calib_result.txt";

    std::ifstream fin(midtxt_path);
    if (!fin.is_open())
    {
        RCLCPP_ERROR(logger, "Failed to open txt file: %s", midtxt_path.c_str());
        rclcpp::shutdown();
        return 1;
    }
    std::vector<std::string> lines;
    for (std::string line; std::getline(fin, line); )
    {
        if (!line.empty()) lines.push_back(line);
    }
    fin.close();
    if (lines.size() < 9) {
        RCLCPP_ERROR(logger, "File has fewer than 9 lines, cannot get 3 blocks.");
        rclcpp::shutdown();
        return 1;
    }

    std::vector<Block> blocks;
    for (size_t i = 0; i + 2 < lines.size(); ++i) 
    {
        if (lines[i].rfind("time:", 0) == 0 &&
            lines[i+1].find("lidar_centers:") != std::string::npos &&
            lines[i+2].find("qr_centers:")    != std::string::npos) 
        {
            Block b;
            b.time_line = lines[i];

            if (!parseCentersLine(lines[i+1], b.lidar_pts)) continue;
            if (!parseCentersLine(lines[i+2], b.qr_pts))    continue;
            if (b.lidar_pts.size() == 4 && b.qr_pts.size() == 4) 
            {
                blocks.push_back(std::move(b));
                i += 2;
            }
        }
    }
    if (blocks.size() < 3) 
    {
        RCLCPP_ERROR(logger, "Parsed blocks < 3 (got %zu).", blocks.size());
        rclcpp::shutdown();
        return 1;
    }

    std::vector<Eigen::Vector3d> L, C;
    for (size_t k = blocks.size() - 3; k < blocks.size(); ++k) 
    {
        const auto& b = blocks[k];
        for (int i = 0; i < 4; ++i) 
        {
            L.push_back(b.lidar_pts[i]);
            C.push_back(b.qr_pts[i]);
        }
    }
    if (L.size() != 12 || C.size() != 12) {
        RCLCPP_ERROR(logger, "Merged pairs not equal to 12 (L=%zu, C=%zu).", L.size(), C.size());
        rclcpp::shutdown();
        return 1;
    }

    std::cout << "LiDAR centers:" << std::endl;
    for (size_t i = 0; i < L.size(); ++i) {
        std::cout << "L[" << i << "]: (" << L[i](0) << ", " << L[i](1) << ", " << L[i](2) << ")" << std::endl;
    }
    std::cout << "QR centers:" << std::endl;
    for (size_t i = 0; i < C.size(); ++i) {
        std::cout << "C[" << i << "]: (" << C[i](0) << ", " << C[i](1) << ", " << C[i](2) << ")" << std::endl;
    }

    auto res = SolveRigidTransformWeighted(L, C, nullptr);
    if (!res.ok) {
        RCLCPP_ERROR(logger, "SolveRigidTransformWeighted failed.");
        rclcpp::shutdown();
        return 1;
    }

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = res.R;
    T.block<3,1>(0,3) = res.t;

    std::cout << BOLDYELLOW << "[Result] RMSE: " << BOLDRED << std::fixed << std::setprecision(4)
      << res.rms << " m" << RESET << std::endl;

    std::cout << BOLDYELLOW << "[Result] Multi-scene calibration: extrinsic parameters T_cam_lidar = " << RESET << std::endl;
    std::cout << BOLDCYAN << std::fixed << std::setprecision(6) << T << RESET << std::endl;

    std::ofstream fout(multi_output_path);
    if (fout.is_open()) 
    {
        fout << "# FAST-LIVO2 calibration format\n";
        fout << std::fixed << std::setprecision(6);
        fout << "Rcl: [ "
            << std::setw(9) << res.R(0,0) << ", " << std::setw(9) << res.R(0,1) << ", " << std::setw(9) << res.R(0,2) << ",\n"
            << "      " << std::setw(9) << res.R(1,0) << ", " << std::setw(9) << res.R(1,1) << ", " << std::setw(9) << res.R(1,2) << ",\n"
            << "      " << std::setw(9) << res.R(2,0) << ", " << std::setw(9) << res.R(2,1) << ", " << std::setw(9) << res.R(2,2) << "]\n";
        fout << "Pcl: [ "
            << std::setw(9) << res.t(0) << ", " << std::setw(9) << res.t(1) << ", " << std::setw(9) << res.t(2) << "]\n";
        fout.close();
        std::cout << BOLDYELLOW << "[Result] Multi-scene calibration results saved to " << BOLDWHITE << multi_output_path << RESET << std::endl;
    } else {
        RCLCPP_WARN(logger, "Failed to write out file: %s", multi_output_path.c_str());
    }

    rclcpp::shutdown();
    return 0;
}
