#ifndef _BASE_ESTIMATION_H_
#define _BASE_ESTIMATION_H_

#include <xbot2/xbot2.h>

#include <ros/ros.h>
#include <xbot2/ros/ros_support.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <base_estimation/base_estimation.h>

namespace XBot {

class BaseEstimationPlugin : public ControlPlugin
{
public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;
    void on_start() override;
    void run() override;
    void on_stop() override;

private:

    typedef std::pair<Eigen::VectorXd, Eigen::VectorXd> ModelState;

    void publishToROS(const Eigen::Affine3d& T, const Eigen::Vector6d& v);
    void convert(const geometry_msgs::TransformStamped& T, geometry_msgs::PoseStamped& P);
    std::vector<std::string> footFrames(const std::string& foot_prefix);

    ModelInterface::Ptr _model;
    ImuSensor::ConstPtr _imu;
    ikbe::BaseEstimation::UniquePtr _est;

    RosSupport::UniquePtr _ros;
    PublisherPtr<tf2_msgs::TFMessage> _base_tf_pub;
    PublisherPtr<geometry_msgs::PoseStamped> _base_pose_pub;
    PublisherPtr<geometry_msgs::TwistStamped> _base_twist_pub;

    PublisherPtr<ModelState> _model_state_pub;
    ModelState _model_state_msg;

};

}

#endif
