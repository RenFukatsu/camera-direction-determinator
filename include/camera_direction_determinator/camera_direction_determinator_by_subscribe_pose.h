#ifndef MULTI_ROBOTS_CAMERA_DIRECTION_DETERMINATOR_H_
#define MULTI_ROBOTS_CAMERA_DIRECTION_DETERMINATOR_H_

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "color_detector_params/hsv.h"
#include "color_detector_srvs/ColorEnable.h"
#include "dynamixel_angle_msgs/DynamixelAngle.h"

class CameraDirectionDeteminatorBySubscribePose {
 public:
    CameraDirectionDeteminatorBySubscribePose();
    void call_color_enable_service(ros::ServiceClient*, std::map<std::string, bool>*, std::string);
    void publish_angle(double radian, int id);
    void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose, int id);
    void set_color_map();
    void process();
    void read_target_roombas();
    void determine_direction();
    bool is_observable(const geometry_msgs::PoseWithCovarianceStamped& p,
                       const geometry_msgs::PoseWithCovarianceStamped& q);
    double calc_direction_angle(const geometry_msgs::PoseWithCovarianceStamped& target,
                                const geometry_msgs::PoseWithCovarianceStamped& source);

 private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::vector<ros::Publisher> dynamixel_pubs_;
    std::vector<ros::Subscriber> pose_subs_;
    std::vector<ros::ServiceClient> color_enable_clients_;
    std::vector<std::string> colors_;
    std::vector<std::map<std::string, bool>> color_enables_;
    std::map<int, bool> target_roombas_;
    std::vector<geometry_msgs::PoseWithCovarianceStamped> roomba_poses_;

    int HZ;
    double OBSERVABLE_DISTANCE;
    double TOLERANCE_TIME_DIFF;
    ros::Publisher ellipse_pub_;
    std::map<std::string, std_msgs::ColorRGBA> color_map_;
};

#endif  // MULTI_ROBOTS_CAMERA_DIRECTION_DETERMINATOR_H_
