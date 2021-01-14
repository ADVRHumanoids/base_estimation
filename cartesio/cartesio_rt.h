#ifndef CARTESIO_RT_H
#define CARTESIO_RT_H

#include <xbot2/xbot2.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <xbot2/gazebo/dev_link_state_sensor.h>

namespace XBot {

class CartesioRt : public ControlPlugin
{

public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;
    void on_start() override;
    void starting() override;
    void run() override;
    void stopping() override;
    void on_abort() override;
    void on_close() override;

private:

    typedef std::pair<Eigen::VectorXd, Eigen::VectorXd> ModelState;

    void on_model_state_recv(const ModelState& msg);

    std::unique_ptr<ros::NodeHandle> _nh;

    bool _enable_feedback;

    JointIdMap _qmap;
    Eigen::VectorXd _q, _qdot;
    ModelInterface::Ptr _rt_model;
    Cartesian::CartesianInterfaceImpl::Ptr _rt_ci;
    Cartesian::LockfreeBufferImpl::Ptr _nrt_ci;
    std::atomic_bool _rt_active;
    std::atomic_bool _nrt_exit;
    double _fake_time;

    std::shared_ptr<Hal::LinkStateSensor> _fb_truth;

    SubscriberBase::Ptr _model_state_sub;
    bool _model_state_recv;

    std::unique_ptr<thread> _nrt_th;
};


}

#endif // CARTESIO_RT_H
