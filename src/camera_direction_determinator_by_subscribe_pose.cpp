#include "camera_direction_determinator/camera_direction_determinator_by_subscribe_pose.h"

CameraDirectionDeteminatorBySubscribePose::CameraDirectionDeteminatorBySubscribePose()
    : private_nh_("~"), tf_listener_(tf_buffer_) {
    private_nh_.param("HZ", HZ, 10);
    private_nh_.param("OBSERVABLE_DISTANCE", OBSERVABLE_DISTANCE, 8.0);
    private_nh_.param("TOLERANCE_TIME_DIFF", TOLERANCE_TIME_DIFF, 1.5);

    std::vector<color_detector_params_hsv::ThresholdHSV> _;
    color_detector_params_hsv::init(colors_, _);
    dynamixel_pubs_.resize(colors_.size());
    pose_subs_.resize(colors_.size());
    roomba_poses_.resize(colors_.size());
    color_enable_clients_.resize(colors_.size());
    color_enables_.resize(colors_.size());
    for (size_t i = 0; i < colors_.size(); i++) {
        std::string roomba = "roomba" + std::to_string(i + 1);
        dynamixel_pubs_[i] = nh_.advertise<dynamixel_angle_msgs::DynamixelAngle>(roomba + "/dynamixel/angle", 1);
        pose_subs_[i] = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
            roomba + "/amcl_pose", 1,
            boost::bind(&CameraDirectionDeteminatorBySubscribePose::pose_callback, this, _1, i));
        color_enable_clients_[i] = nh_.serviceClient<color_detector_srvs::ColorEnable>(roomba + "/color_enable");
        for (const auto &color : colors_) {
            color_enables_[i][color] = false;
        }
    }
    read_target_roombas();
}

void CameraDirectionDeteminatorBySubscribePose::read_target_roombas() {
    for (int i = 0; i < colors_.size(); i++) {
        target_roombas_[i] = false;
    }
    XmlRpc::XmlRpcValue param_list;
    std::string param_name = "TARGET_ROOMBAS";
    if (!private_nh_.getParam(param_name.c_str(), param_list)) {
        for (int i = 0; i < colors_.size(); i++) {
            target_roombas_[i] = true;
        }
        return;
    }
    ROS_ASSERT(param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (size_t i = 0; i < param_list.size(); i++) {
        ROS_ASSERT(param_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
        target_roombas_[param_list[i]] = true;
    }
}

void CameraDirectionDeteminatorBySubscribePose::pose_callback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose, int id) {
    roomba_poses_[id] = *pose;
    return;
}

void CameraDirectionDeteminatorBySubscribePose::publish_angle(double radian, int id) {
    dynamixel_angle_msgs::DynamixelAngle msg;
    msg.theta = radian;
    dynamixel_pubs_[id].publish(msg);
}

void CameraDirectionDeteminatorBySubscribePose::call_color_enable_service(ros::ServiceClient *client,
                                                                          std::map<std::string, bool> *color_enable,
                                                                          std::string color) {
    for (auto itr = color_enable->begin(); itr != color_enable->end(); itr++) {
        if (itr->first == color && itr->second == false) {
            color_detector_srvs::ColorEnable srv;
            srv.request.color = itr->first;
            srv.request.color = true;
            if (client->call(srv)) {
                itr->second = true;
            } else {
                ROS_ERROR_STREAM("Failed to call service color_enable. Couldn't activate " << itr->first << ".");
            }
        }
        if (itr->first != color && itr->second == true) {
            color_detector_srvs::ColorEnable srv;
            srv.request.color = itr->first;
            srv.request.color = false;
            if (client->call(srv)) {
                itr->second = false;
            } else {
                ROS_ERROR_STREAM("Failed to call service color_enable. Couldn't deactivate " << itr->first << ".");
            }
        }
    }
}

bool CameraDirectionDeteminatorBySubscribePose::is_observable(const geometry_msgs::PoseWithCovarianceStamped &p,
                                                              const geometry_msgs::PoseWithCovarianceStamped &q) {
    double px = p.pose.pose.position.x;
    double py = p.pose.pose.position.y;
    double qx = q.pose.pose.position.x;
    double qy = q.pose.pose.position.y;
    return (px - qx) * (px - qx) + (py - qy) * (py - qy) < OBSERVABLE_DISTANCE * OBSERVABLE_DISTANCE;
}

double CameraDirectionDeteminatorBySubscribePose::calc_direction_angle(
    const geometry_msgs::PoseWithCovarianceStamped &target, const geometry_msgs::PoseWithCovarianceStamped &source) {
    double tx = target.pose.pose.position.x;
    double ty = target.pose.pose.position.y;
    double sx = source.pose.pose.position.x;
    double sy = source.pose.pose.position.y;
    double theta = std::atan2(ty - sy, tx - sx);
    tf2::Quaternion quat;
    tf2::convert(source.pose.pose.orientation, quat);
    double r, p, y;
    tf2::Matrix3x3(quat).getRPY(r, p, y);
    double radian = theta - y;
    if (radian < 0) {
        radian += 2 * M_PI;
    }

    return radian;
}

void CameraDirectionDeteminatorBySubscribePose::determine_direction() {
    static std::vector<int64_t> directed_nums(colors_.size(), 0);
    for (int i = 0; i < colors_.size(); i++) {
        if (target_roombas_.count(i) == 1) {  // iがターゲットのルンバであったら、
            continue;
        }
        int target = -1;
        for (int j = 0; j < colors_.size(); j++) {
            if (target_roombas_.count(j) == 0) {  // jがターゲットのルンバでなかったら、
                continue;
            }
            if (!is_observable(roomba_poses_[i], roomba_poses_[j])) {
                continue;
            }
            if ((roomba_poses_[i].header.stamp - roomba_poses_[j].header.stamp).toSec() > TOLERANCE_TIME_DIFF) {
                continue;
            }
            if (target == -1) {
                target = j;
            } else if (directed_nums[target] > directed_nums[j]) {  // より観測が少ない方に向ける
                target = j;
            }
        }
        if (target != -1) {
            call_color_enable_service(&color_enable_clients_[i], &color_enables_[i], colors_[target]);
            double radian = calc_direction_angle(roomba_poses_[target], roomba_poses_[i]);
            publish_angle(radian, i);
            directed_nums[target]++;
            ROS_INFO_STREAM("roomba" << i << "(" << colors_[i] << ") directed to roomba" << target << "(" << colors_[i]
                                     << ").");
        }
    }
}

void CameraDirectionDeteminatorBySubscribePose::process() {
    ros::Rate loop_rate(HZ);

    while (ros::ok()) {
        ros::Time start_time = ros::Time::now();
        determine_direction();
        ROS_INFO_STREAM("[camera_direction_determinator_by_subscribe_pose] elapsed time : "
                        << (ros::Time::now() - start_time).toSec() << "[s]");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_direction_deteminator");
    CameraDirectionDeteminatorBySubscribePose cdd;
    cdd.process();
    return 0;
}
