#ifndef _BASE_ESTIMATION_H_
#define _BASE_ESTIMATION_H_

#include <xbot2/xbot2.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <ros/ros.h>
#include <xbot2/ros/ros_support.h>
#include <tf2_msgs/TFMessage.h>
#include <cartesian_interface/problem/Postural.h>
#include <geometry_msgs/PoseStamped.h>

namespace XBot {

class BaseEstimation : public ControlPlugin
{
public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;
    void on_start() override;
    void run() override;
    void on_stop() override;

private:

    JointIdMap _qmap;
    Eigen::VectorXd _q, _qdot;
    ModelInterface::Ptr _model;
    Cartesian::CartesianInterfaceImpl::Ptr _ci;
    Cartesian::PosturalTask::Ptr _postural;
    Cartesian::CartesianTask::Ptr _imu_task;

    void publishToROS(const Eigen::Affine3d& T);
    void convert(const geometry_msgs::TransformStamped& T, geometry_msgs::PoseStamped& P);

    RosSupport::UniquePtr _ros;
    PublisherPtr<tf2_msgs::TFMessage> _base_tf_pub;
    PublisherPtr<geometry_msgs::PoseStamped> _base_pose_pub;

    bool _use_imu;
    XBot::ImuSensor::ConstPtr _imu;
    Eigen::Matrix3d _w_R_imu;
    Eigen::Vector3d _v_imu;

};

}

#endif
