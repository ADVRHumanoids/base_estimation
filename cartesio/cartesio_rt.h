#ifndef CARTESIO_RT_H
#define CARTESIO_RT_H

#include <xbot2/xbot2.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/sdk/rt/LockfreeBufferImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <xbot2/gazebo/dev_link_state_sensor.h>

#include <base_estimation/ContactsStatus.h>
#include <xbot2/ros/ros_support.h>

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
    void on_contact_state_recv(const base_estimation::ContactsStatus& msg);

    std::unique_ptr<ros::NodeHandle> _nh;
    RosSupport::UniquePtr _ros;

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

    SubscriberBase::Ptr _model_state_sub, _contacts_state_sub;
    bool _model_state_recv;
    std::map<std::string, bool> _contact_state_map;

    std::unique_ptr<thread> _nrt_th;
};


}

#endif // CARTESIO_RT_H
