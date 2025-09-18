#include <ros/ros.h>
#include <ros/service.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros_volume_estimation/ToggleProcessing.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <iostream>

class WorkflowControlNode {
public:
    WorkflowControlNode() {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");
        
        toggle_service_ = nh.advertiseService("/volume_estimation/toggle_processing", 
                                           &WorkflowControlNode::toggleProcessingCallback, this);
        
        status_pub_ = nh.advertise<std_msgs::String>("/volume_estimation/status", 1, true);
        
        processing_enabled_ = false;
        processing_active_ = false;
        
        std_msgs::String status_msg;
        status_msg.data = "STOPPED";
        status_pub_.publish(status_msg);
        
        ROS_INFO("Workflow control node initialized. Service available at: /volume_estimation/toggle_processing");
    }
    
    ~WorkflowControlNode() {
        stopProcessing();
    }
    
private:
    ros::ServiceServer toggle_service_;
    ros::Publisher status_pub_;
    
    bool processing_enabled_;
    bool processing_active_;
    std::vector<pid_t> node_pids_;
    
    bool toggleProcessingCallback(ros_volume_estimation::ToggleProcessing::Request &req,
                                ros_volume_estimation::ToggleProcessing::Response &res) {
        
        if (req.enabled && !processing_enabled_) {
            res.success = startProcessing();
            if (res.success) {
                res.message = "Processing pipeline started successfully";
                processing_enabled_ = true;
                std_msgs::String status_msg;
                status_msg.data = "RUNNING";
                status_pub_.publish(status_msg);
                ROS_INFO("Volume estimation pipeline started");
            } else {
                res.message = "Failed to start processing pipeline";
                ROS_ERROR("Failed to start volume estimation pipeline");
            }
        } else if (!req.enabled && processing_enabled_) {
            res.success = stopProcessing();
            if (res.success) {
                res.message = "Processing pipeline stopped successfully";
                processing_enabled_ = false;
                std_msgs::String status_msg;
                status_msg.data = "STOPPED";
                status_pub_.publish(status_msg);
                ROS_INFO("Volume estimation pipeline stopped");
            } else {
                res.message = "Failed to stop processing pipeline";
                ROS_ERROR("Failed to stop volume estimation pipeline");
            }
        } else {
            res.success = true;
            if (req.enabled) {
                res.message = "Processing pipeline already running";
            } else {
                res.message = "Processing pipeline already stopped";
            }
        }
        
        return true;
    }
    
    bool startProcessing() {
        if (processing_active_) {
            return false;
        }
        
        std::vector<std::string> node_commands = {
            "rosrun ros_volume_estimation transform_node",
            "rosrun ros_volume_estimation crop_filter_node",
            "rosrun ros_volume_estimation volume_estimator_node"
        };
        
        for (const auto& cmd : node_commands) {
            pid_t pid = fork();
            if (pid == 0) {
                execl("/bin/bash", "bash", "-c", cmd.c_str(), (char*)NULL);
                exit(1);
            } else if (pid > 0) {
                node_pids_.push_back(pid);
            } else {
                ROS_ERROR("Failed to fork process for command: %s", cmd.c_str());
                stopProcessing();
                return false;
            }
        }
        
        processing_active_ = true;
        
        ros::Duration(2.0).sleep();
        
        for (pid_t pid : node_pids_) {
            int status;
            pid_t result = waitpid(pid, &status, WNOHANG);
            if (result != 0) {
                ROS_ERROR("Node process %d failed to start properly", pid);
                stopProcessing();
                return false;
            }
        }
        
        return true;
    }
    
    bool stopProcessing() {
        if (!processing_active_) {
            return true;
        }
        
        for (pid_t pid : node_pids_) {
            kill(pid, SIGTERM);
        }
        
        ros::Duration(1.0).sleep();
        
        for (pid_t pid : node_pids_) {
            int status;
            pid_t result = waitpid(pid, &status, WNOHANG);
            if (result == 0) {
                kill(pid, SIGKILL);
                waitpid(pid, &status, 0);
            }
        }
        
        node_pids_.clear();
        processing_active_ = false;
        
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "workflow_control_node");
    
    WorkflowControlNode control_node;
    
    ROS_INFO("Workflow control node ready. Use service calls to start/stop processing.");
    
    ros::spin();
    
    return 0;
}