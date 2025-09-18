#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/convex_hull.h>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>

typedef pcl::PointXYZ PointT;

struct SceneArea {
    int group1_id = 0;
    int group2_id = 0;
    std::vector<PointT> vertexs;
};

class CropFilterNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    
    std::vector<SceneArea> scene_areas_;
    
public:
    CropFilterNode() : nh_("~") {
        // Get polygon parameters
        std::string polygons_str;
        if (!nh_.getParam("polygons", polygons_str)) {
            ROS_ERROR("polygons parameter is required");
            ros::shutdown();
            return;
        }
        
        // Parse scene areas
        setSceneAreas(polygons_str);
        
        // Subscribe and advertise
        sub_ = nh_.subscribe("/pointcloud_transformed", 1, &CropFilterNode::pointCloudCallback, this);
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pointcloud_cropped", 1);
        
        ROS_INFO("Crop Filter Node initialized with %zu scene areas", scene_areas_.size());
    }
    
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        // Convert ROS message to PCL
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*msg, *cloud);
        
        ROS_INFO("Received point cloud with %zu points", cloud->size());
        
        if (cloud->empty()) {
            ROS_WARN("Received empty point cloud");
            return;
        }
        
        // Apply crop filter
        pcl::PointCloud<PointT>::Ptr cropped_cloud = cropFilter(cloud);
        
        // Convert back to ROS message and publish
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*cropped_cloud, output_msg);
        output_msg.header = msg->header;
        pub_.publish(output_msg);
        
        ROS_INFO("Published cropped point cloud with %zu points", cropped_cloud->size());
    }
    
private:
    std::string& ltrim(std::string &ss) {
        std::string::iterator p = std::find_if(ss.begin(), ss.end(), 
            [](unsigned char c) { return !std::isspace(c); });
        ss.erase(ss.begin(), p);
        return ss;
    }
    
    std::string& rtrim(std::string &ss) {
        std::string::reverse_iterator p = std::find_if(ss.rbegin(), ss.rend(), 
            [](unsigned char c) { return !std::isspace(c); });
        ss.erase(p.base(), ss.end());
        return ss;
    }
    
    std::string& trim(std::string &st) {
        ltrim(rtrim(st));
        return st;
    }
    
    std::vector<std::string> stringSplit(const std::string& s, const std::string& delim) {
        std::vector<std::string> elems;
        size_t pos = 0;
        size_t len = s.length();
        size_t delim_len = delim.length();
        if (delim_len == 0) return elems;
        
        while (pos < len) {
            size_t find_pos = s.find(delim, pos);
            if (find_pos == std::string::npos) {
                elems.push_back(s.substr(pos, len - pos));
                break;
            }
            elems.push_back(s.substr(pos, find_pos - pos));
            pos = find_pos + delim_len;
        }
        return elems;
    }
    
    void setSceneAreas(const std::string &scene_areas_str) {
        scene_areas_.clear();
        
        ROS_INFO("Parsing scene areas: %s", scene_areas_str.c_str());
        
        std::vector<std::string> vec = stringSplit(scene_areas_str, ";");
        for (size_t i = 0; i < vec.size(); i++) {
            std::string scene = trim(vec[i]);
            ROS_INFO("Processing scene %zu: %s", i, scene.c_str());
            
            if (scene.empty()) {
                ROS_WARN("Empty scene area, skipping");
                continue;
            }
            
            std::vector<std::string> vec2 = stringSplit(scene, ":");
            if (vec2.size() != 2) {
                ROS_WARN("Invalid scene area format '%s', skipping", scene.c_str());
                continue;
            }
            
            std::vector<std::string> vec_group_ids = stringSplit(vec2[0], ",");
            if (vec_group_ids.size() != 2) {
                ROS_WARN("Invalid group_ids format '%s', skipping", vec2[0].c_str());
                continue;
            }
            
            ROS_INFO("Scene %zu, group_ids: %s, vertexs: %s", i, vec2[0].c_str(), vec2[1].c_str());
            
            std::vector<std::string> vec_vertex = stringSplit(vec2[1], ",");
            if (vec_vertex.size() % 3 != 0) {
                ROS_WARN("Invalid vertex format '%s', skipping", vec2[1].c_str());
                continue;
            }
            
            // Add scene area
            SceneArea area;
            area.group1_id = std::atoi(trim(vec_group_ids[0]).c_str());
            area.group2_id = std::atoi(trim(vec_group_ids[1]).c_str());
            
            for (size_t j = 0; j < vec_vertex.size(); j += 3) {
                float x = std::atof(trim(vec_vertex[j + 0]).c_str());
                float y = std::atof(trim(vec_vertex[j + 1]).c_str());
                float z = std::atof(trim(vec_vertex[j + 2]).c_str());
                
                ROS_INFO("Scene %zu, group1_id: %d, group2_id: %d, vertex: (%.3f, %.3f, %.3f)", 
                         i, area.group1_id, area.group2_id, x, y, z);
                
                PointT pt;
                pt.x = x;
                pt.y = y;
                pt.z = z;
                area.vertexs.push_back(pt);
            }
            
            scene_areas_.push_back(area);
        }
        
        ROS_INFO("Parsed %zu scene areas", scene_areas_.size());
    }
    
    bool constructConvexHull(const std::vector<PointT> &vertexs,
                            pcl::PointCloud<PointT> &surface_hull,
                            std::vector<pcl::Vertices> &polygons) {
        // Define convex hull polyhedron
        pcl::PointCloud<PointT> boundingbox;
        for (const auto& vertex : vertexs) {
            boundingbox.push_back(vertex);
        }
        
        // ConvexHull
        pcl::ConvexHull<PointT> hull;
        hull.setInputCloud(boundingbox.makeShared());
        hull.setDimension(3);
        hull.reconstruct(surface_hull, polygons);
        
        return true;
    }
    
    bool cropHullFilter(const pcl::PointCloud<PointT> &cloud_in,
                       const std::vector<PointT> &vertexs,
                       pcl::PointCloud<PointT> &cloud_out,
                       bool is_keep_inner_points = true) {
        // Calculate convex hull vertex sequence
        pcl::PointCloud<PointT> surface_hull;
        std::vector<pcl::Vertices> polygons;
        constructConvexHull(vertexs, surface_hull, polygons);
        
        // CropHull filter
        pcl::CropHull<PointT> bb_filter;
        bb_filter.setDim(3);
        bb_filter.setInputCloud(cloud_in.makeShared());
        bb_filter.setHullIndices(polygons);
        bb_filter.setHullCloud(surface_hull.makeShared());
        
        if (!is_keep_inner_points) {
            bb_filter.setNegative(true);
        }
        
        bb_filter.filter(cloud_out);
        
        return true;
    }
    
    pcl::PointCloud<PointT>::Ptr cropFilter(pcl::PointCloud<PointT>::Ptr cloud) {
        // Convex hull filtering: area cropping
        pcl::PointCloud<PointT>::Ptr cloud_area(new pcl::PointCloud<PointT>());
        
        if (scene_areas_.size() > 0) {
            for (size_t i = 0; i < scene_areas_.size(); i++) {
                pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>());
                cropHullFilter(*cloud, scene_areas_[i].vertexs, *tmp);
                *cloud_area += *tmp;
                
                ROS_INFO("Applied crop filter %zu, result: %zu points", i, tmp->size());
            }
        }
        
        return cloud_area;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "crop_filter_node");
    CropFilterNode node;
    ros::spin();
    return 0;
}