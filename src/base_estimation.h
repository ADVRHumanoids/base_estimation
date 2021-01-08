#ifndef _BASE_ESTIMATION_H_
#define _BASE_ESTIMATION_H_

#include <xbot2/xbot2.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <ros/ros.h>
#include <xbot2/ros/ros_support.h>
#include <tf2_msgs/TFMessage.h>
#include <cartesian_interface/problem/Postural.h>

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

    void publishTf(const Eigen::Affine3d& T);

    RosSupport::UniquePtr _ros;
    PublisherPtr<tf2_msgs::TFMessage> _base_transform_pub;

};

}

#endif
