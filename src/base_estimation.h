#ifndef _BASE_ESTIMATION_H_
#define _BASE_ESTIMATION_H_

#include <xbot2/xbot2.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <ros/ros.h>
#include <xbot2/ros/ros_support.h>
#include <tf2_msgs/TFMessage.h>
#include <cartesian_interface/problem/Postural.h>
#include <geometry_msgs/PoseStamped.h>
#include <OpenSoT/solvers/BackEnd.h>

namespace XBot {
class ContactForceOptimization
{
public:
    typedef std::shared_ptr<ContactForceOptimization> Ptr;

    ContactForceOptimization(const std::string& sensor_frame,
                             const std::vector<std::string>& corner_frames,
                             ModelInterface::ConstPtr model);

    const Eigen::VectorXd& compute(const double Fz, const double Mx, const double My);
    const std::vector<std::string>& getCornerFrames(){ return _corner_frames;}

private:
    OpenSoT::solvers::BackEnd::Ptr _solver;
    ModelInterface::ConstPtr _model;
    std::vector<std::string> _corner_frames;

    Eigen::MatrixXd _H;
    Eigen::MatrixXd _A;
    Eigen::VectorXd _b;

    Eigen::VectorXd _g;

    Eigen::VectorXd _lb;
    Eigen::VectorXd _ub;
};

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
    std::vector<Cartesian::CartesianTask::Ptr> footFrames(const std::string& foot_prefix);

    RosSupport::UniquePtr _ros;
    PublisherPtr<tf2_msgs::TFMessage> _base_tf_pub;
    PublisherPtr<geometry_msgs::PoseStamped> _base_pose_pub;

    bool _use_imu;
    ImuSensor::ConstPtr _imu;
    Eigen::Matrix3d _w_R_imu;
    Eigen::Vector3d _v_imu;

    std::map<std::string, ForceTorqueSensor::ConstPtr> _ft_map;
    std::map<std::string, std::vector<Cartesian::CartesianTask::Ptr>> _map_foot_cartesian_tasks;
    std::map<std::string, ContactForceOptimization::Ptr> _map_foot_contact_forces;

};

}

#endif
