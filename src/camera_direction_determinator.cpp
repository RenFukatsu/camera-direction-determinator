#include "camera-direction-determinator/camera_direction_determinator.h"

CameraDirectionDeterminator::CameraDirectionDeterminator()
    : private_nh_("~"), tf_listener_(tf_buffer_), start_time_(ros::Time::now()) {
    private_nh_.param("HZ", HZ, 10);
    private_nh_.param("MOTION_NOISE", MOTION_NOISE, 0.1);
    private_nh_.param("MEASUREMENT_NOISE", MEASUREMENT_NOISE, 0.5);

    std::vector<color_detector_params_hsv::ThresholdHSV> _;
    color_detector_params_hsv::init(colors_, _);
    dynamixel_pubs_.resize(colors_.size());
    position_subs_.resize(colors_.size());
    angle_subs_.resize(colors_.size());
    for (size_t i = 0; i < colors_.size(); i++) {
        std::string roomba = "roomba" + std::to_string(i + 1);
        dynamixel_pubs_[i] = nh_.advertise<dynamixel_angle_msgs::DynamixelAngle>(roomba + "/dynamixel/angle", 1);
        position_subs_[i] =
            nh_.subscribe(roomba + "/target/position", 1, &CameraDirectionDeterminator::position_callback, this);
        angle_subs_[i] = nh_.subscribe(roomba + "/target/angle", 1, &CameraDirectionDeterminator::angle_callback, this);
    }
}

void CameraDirectionDeterminator::update_kalman_filter(size_t idx,
                                                       const color_detector_msgs::TargetPositionConstPtr &pos) {
    geometry_msgs::TransformStamped transform_stamped;
    std::string roomba = "roomba" + std::to_string(idx + 1);
    try {
        transform_stamped = tf_buffer_.lookupTransform("map", roomba + "/camera_link", ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN_STREAM(ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    geometry_msgs::PoseStamped target_pose;
    calc_target_pose_on_world(roomba, pos, transform_stamped, &target_pose);
    std::string color = colors_[idx];
    if (kalman_filters_.count(color) == 0) {
        kalman_filters_[color].set_motion_noise(MOTION_NOISE);
        kalman_filters_[color].set_measurement_noise(MEASUREMENT_NOISE);
    }
    kalman_filters_[color].update(target_pose.pose.position.x, target_pose.pose.position.y,
                                  (ros::Time::now() - start_time_).toSec());
}

void CameraDirectionDeterminator::calc_target_pose_on_world(std::string roomba,
                                                            const color_detector_msgs::TargetPositionConstPtr &target,
                                                            const geometry_msgs::TransformStamped &transform,
                                                            geometry_msgs::PoseStamped *output_pose) {
    geometry_msgs::PoseStamped target_pose;
    target_pose.header = target->header;
    target_pose.header.frame_id = roomba + "/camera_link";
    target_pose.pose.position.x = target->z;
    target_pose.pose.position.y = -target->x;
    target_pose.pose.position.z = target->y;
    target_pose.pose.orientation.w = 1;
    target_pose.pose.orientation.x = 0;
    target_pose.pose.orientation.y = 0;
    target_pose.pose.orientation.z = 0;

    tf2::doTransform(target_pose, *output_pose, transform);
    return;
}

void CameraDirectionDeterminator::angle_callback(const color_detector_msgs::TargetAngleListConstPtr &angles) {
    if (angles->data.empty()) {
        ROS_WARN("angle list is empty.");
        return;
    }
    color_detector_msgs::TargetAngle angle;
    double min_likelihood = 1e6;
    for (const auto &agl : angles->data) {
        kalman_filters_[agl.color].estimate_update((ros::Time::now() - start_time_).toSec());
        double likelihood = kalman_filters_[agl.color].get_likelihood();
        if (likelihood < min_likelihood) {
            angle = agl;
            min_likelihood = likelihood;
        }
    }
    dynamixel_angle_msgs::DynamixelAngle msg;
    msg.theta = angle.radian;
    dynamixel_pubs_[angles->my_number - 1].publish(msg);
}

void CameraDirectionDeterminator::position_callback(const color_detector_msgs::TargetPositionConstPtr &position) {
    for (size_t i = 0; i < colors_.size(); i++) {
        if (colors_[i] == position->color) {
            update_kalman_filter(i, position);
        }
    }
}

void CameraDirectionDeterminator::process() { ros::spin(); }
