#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <algorithm>
#include <cmath>
#include <memory>
#include <fstream>
#include <sstream>

class VolumeEstimatorNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    
    // Configuration
    double grid_resolution_x_;
    double grid_resolution_y_;
    std::string height_method_;
    bool interpolate_empty_cells_;
    int max_interpolation_passes_;
    
    struct Bounds {
        double min_x, max_x, min_y, max_y;
    };
    
    struct InterpolationInfo {
        bool used_interpolation;
        int passes_used;
        std::string stop_reason;
        int initial_empty_cells;
        int final_empty_cells;
        int filled_cells;
        int convex_hull_area;
        
        InterpolationInfo() : used_interpolation(false), passes_used(0), stop_reason("no_interpolation"),
                             initial_empty_cells(0), final_empty_cells(0), filled_cells(0), convex_hull_area(0) {}
    };
    
    // OBB-based base plane (from corners file)
    bool have_obb_base_normal_ = false;
    Eigen::Vector3d obb_base_normal_ = Eigen::Vector3d(0.0, 0.0, 1.0);
    std::string obb_corners_file_;
    
public:
    VolumeEstimatorNode() : nh_("~") {
        // Get parameters
        nh_.param("grid_resolution_x", grid_resolution_x_, 0.1);
        nh_.param("grid_resolution_y", grid_resolution_y_, 0.1);
        nh_.param("height_method", height_method_, std::string("mean"));
        nh_.param("interpolate_empty_cells", interpolate_empty_cells_, true);
        nh_.param("max_interpolation_passes", max_interpolation_passes_, 1000);
        
        // Try to get corners file path (prefer local param; fallback to global crop_filter_node param)
        nh_.param<std::string>("corners_file", obb_corners_file_, std::string(""));
        if (obb_corners_file_.empty()) {
            if (!ros::param::get("crop_filter_node/corners_file", obb_corners_file_)) {
                ROS_WARN("No corners_file provided for volume_estimator_node, falling back to PCA plane normal.");
            }
        }
        if (!obb_corners_file_.empty()) {
            have_obb_base_normal_ = loadObbCornersAndComputeBaseNormal(obb_corners_file_);
        }
        
        // Subscribe and advertise
        sub_ = nh_.subscribe("/pointcloud_cropped", 1, &VolumeEstimatorNode::pointCloudCallback, this);
        pub_ = nh_.advertise<std_msgs::Float64>("/volume_result", 1);
        
        ROS_INFO("Volume Estimator Node initialized");
        ROS_INFO("Parameters: grid_resolution_x=%.3f, grid_resolution_y=%.3f, height_method=%s", 
                 grid_resolution_x_, grid_resolution_y_, height_method_.c_str());
        ROS_INFO("Interpolation: enabled=%s, max_passes=%d", 
                 interpolate_empty_cells_ ? "true" : "false", max_interpolation_passes_);
    }
    
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        // Convert ROS message to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        ROS_INFO("Received point cloud with %zu points for volume estimation", cloud->size());
        
        if (cloud->empty()) {
            ROS_WARN("Received empty point cloud, volume = 0");
            std_msgs::Float64 volume_msg;
            volume_msg.data = 0.0;
            pub_.publish(volume_msg);
            return;
        }
        
        // Estimate volume
        std::pair<double, InterpolationInfo> result = estimateVolume(cloud);
        double volume = result.first;
        InterpolationInfo interp_info = result.second;
        
        // Publish volume result
        std_msgs::Float64 volume_msg;
        volume_msg.data = volume;
        pub_.publish(volume_msg);
        
        // Print results
        ROS_INFO("=== DEM Volume Estimation Results ===");
        ROS_INFO("Estimated volume: %.6f cubic units", volume);
        
        if (interp_info.used_interpolation) {
            ROS_INFO("=== Interpolation Information ===");
            ROS_INFO("Interpolation passes: %d", interp_info.passes_used);
            ROS_INFO("Stop reason: %s", interp_info.stop_reason.c_str());
            ROS_INFO("Convex hull area: %d grids", interp_info.convex_hull_area);
            ROS_INFO("Initial empty grids: %d", interp_info.initial_empty_cells);
            ROS_INFO("Final empty grids: %d", interp_info.final_empty_cells);
            ROS_INFO("Filled grids: %d", interp_info.filled_cells);
        }
    }
    
private:
    // -------- Helpers to match pcd_tool.py DEM semantics --------
    Eigen::Vector3d computePlaneNormalPCA(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        // Compute mean
        Eigen::Vector3d mean(0.0, 0.0, 0.0);
        for (const auto& p : cloud->points) {
            mean.x() += p.x; mean.y() += p.y; mean.z() += p.z;
        }
        if (cloud->points.empty()) return Eigen::Vector3d(0.0, 0.0, 1.0);
        mean /= static_cast<double>(cloud->points.size());

        // Compute covariance
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (const auto& p : cloud->points) {
            Eigen::Vector3d v(p.x - mean.x(), p.y - mean.y(), p.z - mean.z());
            cov += v * v.transpose();
        }
        if (cloud->points.size() > 1) {
            cov /= static_cast<double>(cloud->points.size() - 1);
        }

        // Eigen decomposition (ascending eigenvalues)
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
        if (solver.info() != Eigen::Success) {
            return Eigen::Vector3d(0.0, 0.0, 1.0);
        }
        Eigen::Vector3d n = solver.eigenvectors().col(0); // smallest eigenvalue => plane normal
        if (n.norm() < 1e-12) return Eigen::Vector3d(0.0, 0.0, 1.0);
        // Ensure pointing upwards (optional)
        if (n.z() < 0) n = -n;
        return n.normalized();
    }

    Eigen::Matrix3d rotationAlignAToB(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        Eigen::Vector3d an = a.normalized();
        Eigen::Vector3d bn = b.normalized();
        Eigen::Vector3d v = an.cross(bn);
        double s = v.norm();
        double c = an.dot(bn);
        if (s < 1e-12) {
            if (c > 0.0) {
                return Eigen::Matrix3d::Identity();
            } else {
                // 180 deg rotation around any axis orthogonal to an
                Eigen::Vector3d axis = an.unitOrthogonal();
                Eigen::Matrix3d K;
                K <<     0, -axis.z(),  axis.y(),
                      axis.z(),       0, -axis.x(),
                     -axis.y(),  axis.x(),       0;
                return Eigen::Matrix3d::Identity() + K + K * K;
            }
        }
        Eigen::Matrix3d K;
        K <<     0, -v.z(),  v.y(),
              v.z(),     0, -v.x(),
             -v.y(),  v.x(),     0;
        return Eigen::Matrix3d::Identity() + K + K * K * ((1.0 - c) / (s * s));
    }

    bool loadObbCornersAndComputeBaseNormal(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            ROS_WARN("Cannot open OBB corners file: %s", filepath.c_str());
            return false;
        }
        std::vector<Eigen::Vector3d> corners;
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            double x, y, z;
            if (iss >> x >> y >> z) {
                corners.emplace_back(x, y, z);
            }
        }
        file.close();
        if (corners.size() < 4) {
            ROS_WARN("OBB corners file contains fewer than 4 points (got %zu)", corners.size());
            return false;
        }
        // Take 4 lowest-z points as bottom face and fit plane normal
        std::sort(corners.begin(), corners.end(), [](const Eigen::Vector3d& a, const Eigen::Vector3d& b){
            return a.z() < b.z();
        });
        size_t take = std::min<size_t>(4, corners.size());
        Eigen::Vector3d mean(0.0, 0.0, 0.0);
        for (size_t i = 0; i < take; ++i) {
            mean += corners[i];
        }
        mean /= static_cast<double>(take);
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (size_t i = 0; i < take; ++i) {
            Eigen::Vector3d v = corners[i] - mean;
            cov += v * v.transpose();
        }
        if (take > 1) {
            cov /= static_cast<double>(take - 1);
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
        if (solver.info() != Eigen::Success) {
            ROS_WARN("Failed to eigen-decompose bottom face covariance");
            return false;
        }
        Eigen::Vector3d n = solver.eigenvectors().col(0);
        if (n.norm() < 1e-12) {
            ROS_WARN("Degenerate bottom normal from corners");
            return false;
        }
        if (n.z() < 0.0) n = -n;
        obb_base_normal_ = n.normalized();
        ROS_INFO("Loaded OBB base normal from corners (%.6f, %.6f, %.6f)", obb_base_normal_.x(), obb_base_normal_.y(), obb_base_normal_.z());
        return true;
    }

    Bounds calculateBounds(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        Bounds bounds;
        if (cloud->points.empty()) return bounds;
        
        bounds.min_x = bounds.max_x = cloud->points[0].x;
        bounds.min_y = bounds.max_y = cloud->points[0].y;
        
        for (const auto& point : cloud->points) {
            bounds.min_x = std::min(bounds.min_x, static_cast<double>(point.x));
            bounds.max_x = std::max(bounds.max_x, static_cast<double>(point.x));
            bounds.min_y = std::min(bounds.min_y, static_cast<double>(point.y));
            bounds.max_y = std::max(bounds.max_y, static_cast<double>(point.y));
        }
        return bounds;
    }
    
    void createHeightMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                        const Bounds& bounds,
                        Eigen::MatrixXd& height_map,
                        Eigen::MatrixXi& count_map,
                        Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>& empty_mask) {
        
        height_map.setZero();
        count_map.setZero();
        
        for (const auto& point : cloud->points) {
            int grid_x = std::min(static_cast<int>((point.x - bounds.min_x) / grid_resolution_x_), 
                                 static_cast<int>(height_map.rows() - 1));
            int grid_y = std::min(static_cast<int>((point.y - bounds.min_y) / grid_resolution_y_), 
                                 static_cast<int>(height_map.cols() - 1));
            
            if (height_method_ == "max") {
                height_map(grid_x, grid_y) = std::max(height_map(grid_x, grid_y), 
                                                     static_cast<double>(point.z));
            } else { // mean
                height_map(grid_x, grid_y) += point.z;
                count_map(grid_x, grid_y)++;
            }
        }
        
        // Process mean method and determine empty grids
        if (height_method_ == "mean") {
            for (int i = 0; i < height_map.rows(); ++i) {
                for (int j = 0; j < height_map.cols(); ++j) {
                    if (count_map(i, j) > 0) {
                        height_map(i, j) /= count_map(i, j);
                        empty_mask(i, j) = false;
                    } else {
                        empty_mask(i, j) = true;
                    }
                }
            }
        } else {
            for (int i = 0; i < height_map.rows(); ++i) {
                for (int j = 0; j < height_map.cols(); ++j) {
                    empty_mask(i, j) = (height_map(i, j) == 0.0);
                }
            }
        }
    }
    
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> getConvexHullMask(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const Bounds& bounds, int num_x, int num_y) {
        
        Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> valid_mask(num_x, num_y);
        
        for (int i = 0; i < num_x; ++i) {
            for (int j = 0; j < num_y; ++j) {
                valid_mask(i, j) = true;
            }
        }
        
        try {
            // Use PCL to calculate convex hull
            pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ConvexHull<pcl::PointXYZ> chull;
            chull.setInputCloud(cloud);
            chull.setDimension(2);  // Set to 2D convex hull
            chull.reconstruct(*hull_points);
            
            // Convert to 2D polygon
            std::vector<std::pair<double, double>> polygon;
            for (const auto& point : hull_points->points) {
                polygon.push_back(std::make_pair(point.x, point.y));
            }
            
            // Check if each grid center is inside the convex hull
            for (int i = 0; i < num_x; ++i) {
                for (int j = 0; j < num_y; ++j) {
                    double center_x = bounds.min_x + (i + 0.5) * grid_resolution_x_;
                    double center_y = bounds.min_y + (j + 0.5) * grid_resolution_y_;
                    valid_mask(i, j) = isPointInPolygon(center_x, center_y, polygon);
                }
            }
            
        } catch (const std::exception& e) {
            ROS_WARN("Failed to calculate convex hull, will interpolate for all areas: %s", e.what());
            for (int i = 0; i < num_x; ++i) {
                for (int j = 0; j < num_y; ++j) {
                    valid_mask(i, j) = true;
                }
            }
        }
        
        return valid_mask;
    }
    
    bool isPointInPolygon(double x, double y, 
                         const std::vector<std::pair<double, double>>& polygon) {
        int count = 0;
        for (size_t i = 0; i < polygon.size(); ++i) {
            size_t j = (i + 1) % polygon.size();
            
            if (((polygon[i].second > y) != (polygon[j].second > y)) &&
                (x < (polygon[j].first - polygon[i].first) * (y - polygon[i].second) / 
                 (polygon[j].second - polygon[i].second) + polygon[i].first)) {
                count++;
            }
        }
        return count % 2 == 1;
    }
    
    std::pair<Eigen::MatrixXd, InterpolationInfo> interpolateHeightMap(
        const Eigen::MatrixXd& height_map,
        const Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>& empty_mask,
        const Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>& valid_mask) {
        
        Eigen::MatrixXd result = height_map;
        Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> has_value_mask(height_map.rows(), height_map.cols());
        
        for (int i = 0; i < height_map.rows(); ++i) {
            for (int j = 0; j < height_map.cols(); ++j) {
                has_value_mask(i, j) = !empty_mask(i, j);
            }
        }
        
        InterpolationInfo info;
        info.used_interpolation = true;
        
        // Calculate statistics
        int convex_hull_count = 0;
        int initial_empty_count = 0;
        
        for (int i = 0; i < height_map.rows(); ++i) {
            for (int j = 0; j < height_map.cols(); ++j) {
                if (valid_mask(i, j)) convex_hull_count++;
                if (empty_mask(i, j) && valid_mask(i, j)) initial_empty_count++;
            }
        }
        
        info.convex_hull_area = convex_hull_count;
        info.initial_empty_cells = initial_empty_count;
        
        std::vector<std::pair<int, int>> interpolated_cells;
        
        for (info.passes_used = 0; info.passes_used < max_interpolation_passes_; info.passes_used++) {
            interpolated_cells.clear();
            
            for (int i = 0; i < result.rows(); ++i) {
                for (int j = 0; j < result.cols(); ++j) {
                    if (!has_value_mask(i, j) && valid_mask(i, j)) {
                        double sum = 0.0;
                        int count = 0;
                        
                        // Check 4 neighbors (up, down, left, right)
                        int neighbors[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
                        for (int k = 0; k < 4; ++k) {
                            int ni = i + neighbors[k][0];
                            int nj = j + neighbors[k][1];
                            
                            if (ni >= 0 && ni < result.rows() && 
                                nj >= 0 && nj < result.cols() && 
                                has_value_mask(ni, nj)) {
                                sum += result(ni, nj);
                                count++;
                            }
                        }
                        
                        if (count > 0) {
                            result(i, j) = sum / count;
                            interpolated_cells.push_back(std::make_pair(i, j));
                        }
                    }
                }
            }
            
            // Update has_value_mask
            for (const auto& pos : interpolated_cells) {
                has_value_mask(pos.first, pos.second) = true;
            }
            
            if (interpolated_cells.empty()) {
                info.stop_reason = "converged";
                break;
            }
        }
        
        if (info.passes_used >= max_interpolation_passes_) {
            info.stop_reason = "max_passes";
        }
        
        // Calculate final statistics
        int final_empty_count = 0;
        for (int i = 0; i < result.rows(); ++i) {
            for (int j = 0; j < result.cols(); ++j) {
                if (!has_value_mask(i, j) && valid_mask(i, j)) {
                    final_empty_count++;
                }
            }
        }
        
        info.final_empty_cells = final_empty_count;
        info.filled_cells = info.initial_empty_cells - info.final_empty_cells;
        
        return std::make_pair(result, info);
    }
    
    std::pair<double, InterpolationInfo> estimateVolume(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        if (cloud->points.empty()) {
            InterpolationInfo empty_info;
            empty_info.stop_reason = "no_points";
            return std::make_pair(0.0, empty_info);
        }

        // 1) Base plane from OBB corners if available; otherwise PCA plane normal
        Eigen::Vector3d n = have_obb_base_normal_ ? obb_base_normal_ : computePlaneNormalPCA(cloud);
        const Eigen::Vector3d zAxis(0.0, 0.0, 1.0);
        Eigen::Matrix3d R = rotationAlignAToB(n, zAxis); // R * n -> +Z

        // 2) Rotate cloud and level by subtracting the minimum z (clamp to >= 0)
        pcl::PointCloud<pcl::PointXYZ>::Ptr rot_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        rot_cloud->points.reserve(cloud->points.size());
        double min_z = std::numeric_limits<double>::infinity();
        for (const auto& p : cloud->points) {
            Eigen::Vector3d v(p.x, p.y, p.z);
            Eigen::Vector3d vr = R * v;
            if (vr.z() < min_z) min_z = vr.z();
            rot_cloud->points.emplace_back(static_cast<float>(vr.x()),
                                           static_cast<float>(vr.y()),
                                           static_cast<float>(vr.z()));
        }
        if (!std::isfinite(min_z)) min_z = 0.0;
        for (auto& p : rot_cloud->points) {
            double z = static_cast<double>(p.z) - min_z;
            if (z < 0.0) z = 0.0;
            p.z = static_cast<float>(z);
        }
        rot_cloud->width = cloud->width;
        rot_cloud->height = cloud->height;
        rot_cloud->is_dense = cloud->is_dense;

        // 3) Calculate bounds on rotated cloud
        Bounds bounds = calculateBounds(rot_cloud);
        // 4) Grid size on rotated XY
        int num_x = std::max(1, static_cast<int>(
            std::ceil((bounds.max_x - bounds.min_x) / grid_resolution_x_)));
        int num_y = std::max(1, static_cast<int>(
            std::ceil((bounds.max_y - bounds.min_y) / grid_resolution_y_)));
        // 5) Create height map using rotated cloud and leveled z
        Eigen::MatrixXd height_map(num_x, num_y);
        Eigen::MatrixXi count_map(num_x, num_y);
        Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> empty_mask(num_x, num_y);
        createHeightMap(rot_cloud, bounds, height_map, count_map, empty_mask);
        
        InterpolationInfo interp_info;
        interp_info.stop_reason = "no_interpolation";
        
        // Perform interpolation
        if (interpolate_empty_cells_ && max_interpolation_passes_ > 0) {
            // Use rotated cloud for convex hull mask to match current grid frame
            Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> valid_mask = getConvexHullMask(rot_cloud, bounds, num_x, num_y);
            std::pair<Eigen::MatrixXd, InterpolationInfo> interp_result = interpolateHeightMap(
                height_map, empty_mask, valid_mask);
            height_map = interp_result.first;
            interp_info = interp_result.second;
        }
        
        // Calculate volume
        double cell_area = grid_resolution_x_ * grid_resolution_y_;
        double total_volume = height_map.sum() * cell_area;
        
        return std::make_pair(total_volume, interp_info);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "volume_estimator_node");
    VolumeEstimatorNode node;
    ros::spin();
    return 0;
}