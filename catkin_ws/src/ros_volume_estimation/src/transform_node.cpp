#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <iomanip>

class TransformNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    
    // Transformation matrix
    Eigen::Matrix4f transformation_matrix_;
    bool use_inverse_;
    bool show_info_;
    
public:
    TransformNode() : nh_("~") {
        // Get transformation matrix from parameters
        std::vector<double> matrix_params;
        if (!nh_.getParam("transformation_matrix", matrix_params) || matrix_params.size() != 16) {
            ROS_ERROR("transformation_matrix parameter must contain exactly 16 elements");
            ros::shutdown();
            return;
        }
        
        // Convert to Eigen matrix
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                transformation_matrix_(i, j) = matrix_params[i * 4 + j];
            }
        }
        
        nh_.param("use_inverse", use_inverse_, false);
        nh_.param("show_info", show_info_, true);
        
        // Subscribe and advertise
        sub_ = nh_.subscribe("/pointcloud_raw", 1, &TransformNode::pointCloudCallback, this);
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pointcloud_transformed", 1);
        
        ROS_INFO("Transform Node initialized");
        if (show_info_) {
            printTransformationInfo();
        }
    }
    
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        // Convert ROS message to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        ROS_INFO("Received point cloud with %zu points", cloud->size());
        
        if (cloud->empty()) {
            ROS_WARN("Received empty point cloud");
            return;
        }
        
        // Apply transformation
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud = transformPointCloud(cloud);
        
        if (transformed_cloud) {
            // Convert back to ROS message and publish
            sensor_msgs::PointCloud2 output_msg;
            pcl::toROSMsg(*transformed_cloud, output_msg);
            output_msg.header = msg->header;
            pub_.publish(output_msg);
            
            ROS_INFO("Published transformed point cloud with %zu points", transformed_cloud->size());
        }
    }
    
private:
    void printTransformationInfo() {
        ROS_INFO("=== Transformation Information ===");
        
        Eigen::Matrix4f matrix_to_show = transformation_matrix_;
        if (use_inverse_) {
            float det = transformation_matrix_.determinant();
            if (std::abs(det) > 1e-6) {
                matrix_to_show = transformation_matrix_.inverse();
                ROS_INFO("Showing inverse transformation matrix information");
            } else {
                ROS_WARN("Transformation matrix is not invertible, showing original matrix");
            }
        }
        
        // Extract rotation matrix and translation vector
        Eigen::Matrix3f rotation_matrix = matrix_to_show.block<3, 3>(0, 0);
        Eigen::Vector3f translation_vector = matrix_to_show.block<3, 1>(0, 3);
        
        ROS_INFO("4x4 Transformation Matrix:");
        for (int i = 0; i < 4; i++) {
            std::ostringstream oss;
            for (int j = 0; j < 4; j++) {
                oss << std::setw(12) << std::fixed << std::setprecision(6) << matrix_to_show(i, j) << " ";
            }
            ROS_INFO("%s", oss.str().c_str());
        }
        
        ROS_INFO("Translation Vector:");
        ROS_INFO("X: %.6f", translation_vector(0));
        ROS_INFO("Y: %.6f", translation_vector(1));
        ROS_INFO("Z: %.6f", translation_vector(2));
        ROS_INFO("Total translation distance: %.4f", translation_vector.norm());
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        // Calculate the transformation matrix to use
        Eigen::Matrix4f final_transformation = transformation_matrix_;
        if (use_inverse_) {
            ROS_INFO("Computing inverse transformation matrix...");
            float det = transformation_matrix_.determinant();
            if (std::abs(det) < 1e-6) {
                ROS_ERROR("Transformation matrix is not invertible");
                return nullptr;
            }
            final_transformation = transformation_matrix_.inverse();
            ROS_INFO("Using inverse transformation matrix");
        } else {
            ROS_INFO("Using original transformation matrix");
        }
        
        // Apply transformation matrix
        ROS_INFO("Applying transformation matrix...");
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud, *transformed_cloud, final_transformation);
        
        ROS_INFO("Transformation completed, transformed point cloud contains %zu points", transformed_cloud->size());
        
        return transformed_cloud;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "transform_node");
    TransformNode node;
    ros::spin();
    return 0;
}