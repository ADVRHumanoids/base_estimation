#ifndef _BASE_ESTIMATION_H_
#define _BASE_ESTIMATION_H_

#include <xbot2/xbot2.h>

#include <ros/ros.h>
#include <xbot2/ros/ros_support.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <base_estimation/base_estimation.h>
#include <xbot2/gazebo/dev_link_state_sensor.h>
#include <matlogger2/matlogger2.h>

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
    typedef std::map<std::string,bool> ContactsState; //if true contact enabled

    void publishToROS(const Eigen::Affine3d& T, const Eigen::Vector6d& v, const Eigen::Vector6d& raw_v);
    void convert(const geometry_msgs::TransformStamped& T, geometry_msgs::PoseStamped& P);

    /**
     * @brief footFrames get frames associated to a foot
     * @param foot_prefix prefix of a foot
     * @return vector of frames associated to a foot_prefix
     */
    std::vector<std::string> footFrames(const std::string& foot_prefix);

    ModelInterface::Ptr _model;
    ImuSensor::ConstPtr _imu;
    ikbe::BaseEstimation::UniquePtr _est;
    std::shared_ptr<Hal::LinkStateSensor> _gz;

    RosSupport::UniquePtr _ros;
    PublisherPtr<tf2_msgs::TFMessage> _base_tf_pub;
    PublisherPtr<geometry_msgs::PoseStamped> _base_pose_pub, _base_pose_gz_pub;
    PublisherPtr<geometry_msgs::TwistStamped> _base_twist_pub, _base_raw_twist_pub, _base_twist_gz_pub;

    PublisherPtr<ModelState> _model_state_pub;
    ModelState _model_state_msg;

    MatLogger2::Ptr _logger;

    ContactsState _contacts_state;
    double _in_contact_ths, _not_in_contact_ths;
    std::map<std::string, ForceTorqueSensor::ConstPtr> _ft_map;

};

}

#endif
