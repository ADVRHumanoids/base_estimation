#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

#include <RobotInterfaceROS/ConfigFromParam.h>
#include <XBotInterface/RobotInterface.h>
#include <matlogger2/matlogger2.h>
#include <xbot2/journal/journal.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>

#include <base_estimation/base_estimation.h>
#include <base_estimation/ContactsStatus.h>
#include <base_estimation/contact_viz.h>

#include "common.h"

using namespace std::string_literals;

class BaseEstimationNode :
        private XBot::Journal
{

public:

    BaseEstimationNode();

    double getRate() const;

    void start();

    bool run();

private:

    ros::NodeHandle _nh;
    ros::NodeHandle _nhpr;

    XBot::RobotInterface::Ptr _robot;
    XBot::ModelInterface::Ptr _model;
    ikbe::BaseEstimation::UniquePtr _est;

    std::unique_ptr<XBot::Cartesian::Utils::RobotStatePublisher> _rspub;
    tf::TransformBroadcaster _br;

    std::string _odom_frame;
    std::string _tf_prefix, _tf_prefix_slash;
    double _pose_lin_cov, _pose_rot_cov;
    double _vel_lin_cov, _vel_rot_cov;

    ros::Publisher _base_tf_pub;
    ros::Publisher _base_pose_pub;
    ros::Publisher _base_twist_pub;
    ros::Publisher _base_odom_pub;
    ros::Publisher _base_raw_twist_pub;
    ros::Publisher _contacts_state_pub;

    ros::Time _last_now;

    void publishToROS(const Eigen::Affine3d& T,
                      const Eigen::Vector6d& v,
                      const Eigen::Vector6d& raw_v);

};

BaseEstimationNode::BaseEstimationNode():
    XBot::Journal(XBot::Journal::no_publish,
                  "base_estimation_node"),
    _nhpr("~")
{
    // get config options
    auto cfg = XBot::ConfigOptionsFromParamServer();

    // get robot and model
    _robot = XBot::RobotInterface::getRobot(cfg);
    _model = XBot::ModelInterface::getModel(cfg);

    // check if floating base
    if(!_model->isFloatingBase())
    {
        throw std::runtime_error("model is not floating base");
    }

    // set model to the robot state
    _robot->sense(false);
    _model->syncFrom(*_robot);

    // rspub
    if(_nhpr.param<bool>("publish_tf", false))
    {
        _rspub = std::make_unique<XBot::Cartesian::Utils::RobotStatePublisher>(_model);
    }

    if(!_nhpr.getParam("tf_prefix", _tf_prefix))
    {
        _tf_prefix = "odometry";
    }

    _tf_prefix_slash = _tf_prefix.empty() ? "" : _tf_prefix_slash;

    // load problem
    std::string ik_problem_str;
    if(!_nhpr.getParam("ik_problem", ik_problem_str))
    {
        throw std::runtime_error("~ik_problem param missing");
    }

    auto ik_problem_yaml = YAML::Load(ik_problem_str);

    // estimator options
    ikbe::BaseEstimation::Options est_opt;
    est_opt.dt = 1./_nhpr.param("rate", 100.0);
    est_opt.log_enabled = _nhpr.param("enable_log", false);
    est_opt.contact_attach_thr = _nhpr.param("contact_attach_thr", 40.0);
    est_opt.contact_release_thr = _nhpr.param("contact_release_thr", 10.0);

    // create estimator
    _est = std::make_unique<ikbe::BaseEstimation>(_model,
                                                  ik_problem_yaml,
                                                  est_opt);

    // use imu
    if(_nhpr.param("use_imu", false))
    {
        if(!_robot->getImu().empty())
        {
            auto imu = _robot->getImu().begin()->second;
            _est->addImu(imu);
            jinfo("using imu '{}'", imu->getSensorName());
        }
        else
        {
            throw std::runtime_error("no imu defined");
        }
    }

    // set world frame coincident to given link
    std::string world_from_tf = _nhpr.param("world_from_tf", ""s);
    if(world_from_tf != "")
    {
        tf::TransformListener tl;

        std::string floating_base_link;
        _model->getFloatingBaseLink(floating_base_link);

        std::string err;
        if(!tl.waitForTransform(world_from_tf,
                                floating_base_link,
                                ros::Time(0), ros::Duration(5.0), ros::Duration(0.01), &err))
        {
            ROS_ERROR("tf lookup failed: %s", err.c_str());
            exit(1);
        }

        tf::StampedTransform tf;
        tl.lookupTransform(world_from_tf,
                           floating_base_link,
                           ros::Time(0),
                           tf);


        Eigen::Affine3d fb_T_l;
        tf::transformTFToEigen(tf, fb_T_l);

        _model->setFloatingBasePose(fb_T_l);
        _model->update();
    }

    // set world frame from given tf
    std::string world_frame_link = _nhpr.param("world_frame_link", ""s);
    if(world_frame_link != "")
    {

        Eigen::Affine3d fb_T_l;
        std::string floating_base_link;
        _model->getFloatingBaseLink(floating_base_link);
        if(!_model->getPose(world_frame_link, floating_base_link, fb_T_l))
        {
            throw std::runtime_error("world frame link '" + world_frame_link + "' is undefined");
        }

        jinfo("using link '{}' as world frame", world_frame_link);

        _model->setFloatingBasePose(fb_T_l.inverse());
        _model->update();
    }

    // get contact properties

    // rolling contacts (ft name -> wheel name map)
    std::map<std::string, std::string> rolling_contacts;

    // get it from parameters
    _nhpr.getParam("rolling_contacts", rolling_contacts);

    // z force override
    std::map<std::string, double> z_force_override;
    _nhpr.getParam("z_force_override", z_force_override);

    for(auto rc : rolling_contacts)
    {
        auto ft_name = rc.first;
        auto wh_name = rc.second;

        // map of available ft sensors
        auto ft_map = _robot->getForceTorque();

        // ft for contact detection
        XBot::ForceTorqueSensor::ConstPtr ft;

        // add ft (either real or virtual)
        if(z_force_override.count(ft_name))
        {
            auto dummy_ft = ikbe::BaseEstimation::CreateDummyFtSensor(ft_name);
            dummy_ft->setForce(z_force_override.at(ft_name) * Eigen::Vector3d::UnitZ(),
                               0.0);  // useless timestamp
            jinfo("created dummy ft {} for wheel {}, fz = {}",
                  ft_name, wh_name, z_force_override.at(ft_name));
            ft = dummy_ft;
        }
        else if(ft_map.count(ft_name) > 0)
        {
            ft = ft_map.at(ft_name);
        }
        else
        {
            ft = _est->createVirtualFt(ft_name, {0, 1, 2});
        }

        // create contact
        _est->addRollingContact(wh_name, ft);

        jinfo("adding rolling contact (ft: '{}', wheel: '{}')",
              ft_name,
              wh_name);
    }

    // surface contacts (including point contacts)
    std::map<std::string, std::string> surface_contacts;

    // get it from parameters
    _nhpr.getParam("surface_contacts", surface_contacts);

    for(auto sc : surface_contacts)
    {
        // save force-torque name
        auto ft_name = sc.first;
        auto vertex_prefix = sc.second;

        // retrieve foot corner frames based on
        // the given prefix
        auto vertices = ikbe_common::footFrames(*_est->ci(),
                                                vertex_prefix);

        // map of available ft sensors
        auto ft_map = _robot->getForceTorque();

        // ft for contact detection
        XBot::ForceTorqueSensor::ConstPtr ft;

        // add ft (either real or virtual)
        if(ft_map.count(ft_name) > 0)
        {
            ft = ft_map.at(ft_name);
        }
        else
        {
            ft = _est->createVirtualFt(ft_name, {0, 1, 2});
        }

        // create contact
        _est->addSurfaceContact(vertices, ft);

        jinfo("adding surface contact '{}' with vertices: [{}]",
              ft_name,
              fmt::join(vertices, ", "));
    }

    // publishers
    _base_tf_pub = _nhpr.advertise<tf2_msgs::TFMessage>("/tf", 1);
    _base_pose_pub = _nhpr.advertise<geometry_msgs::TransformStamped>("base_link/pose", 1);
    _base_twist_pub = _nhpr.advertise<geometry_msgs::TwistStamped>("base_link/twist", 1);
    _base_raw_twist_pub = _nhpr.advertise<geometry_msgs::TwistStamped>("base_link/raw_twist", 1);
    _contacts_state_pub = _nhpr.advertise<base_estimation::ContactsStatus>("contacts/status", 1);
    _base_odom_pub = _nhpr.advertise<nav_msgs::Odometry>("base_link/odom", 1);

    // odom frame name
    _odom_frame = _nhpr.param("odom_frame", "world"s);

    // covariance
    _pose_lin_cov = _nhpr.param("pose_lin_cov", 1.0);
    _pose_rot_cov = _nhpr.param("pose_rot_cov", 1.0);
    _vel_lin_cov = _nhpr.param("vel_lin_cov", 1.0);
    _vel_rot_cov = _nhpr.param("vel_rot_cov", 1.0);

    // filter params
    double filter_param = 0.;
    if(_nhpr.getParam("velocity_filter/omega", filter_param))
    {
        _est->setFilterOmega(_nhpr.param("velocity_filter/omega", 1e3));
    }
    if(_nhpr.getParam("filter_damping", filter_param))
    {
        _est->setFilterDamping(filter_param);
    }

    _est->setFilterTs(est_opt.dt);
}

double BaseEstimationNode::getRate() const
{
    return 1./_est->getOptions().dt;
}

void BaseEstimationNode::start()
{
    _robot->sense(false);
    _model->syncFrom(*_robot);

    if(_est->imu())
    {
        jinfo("resetting model from imu");
        _model->setFloatingBaseState(_est->imu());
    }

    _model->update();

    _est->reset();
}

bool BaseEstimationNode::run()
{
    // update robot
    _robot->sense(false);
    _model->syncFrom(*_robot);

    // update estimate
    Eigen::Affine3d base_pose;
    Eigen::Vector6d base_vel, raw_base_vel;
    if(!_est->update(base_pose, base_vel, raw_base_vel))
    {
        jerror("unable to solve");
        return false;
    }

    // publish contact markers in ROS
    // tbd publishVertexWeights();
    // tbd publishContactStatus();

    // base state broadcast in ROS
    publishToROS(base_pose, base_vel, raw_base_vel);

    return true;
}

void BaseEstimationNode::publishToROS(const Eigen::Affine3d& T,
                                      const Eigen::Vector6d& v,
                                      const Eigen::Vector6d& raw_v)
{
    // protect against duplicated tf warning
    auto now = ros::Time::now();

    if(now == _last_now)
    {
        return;
    }

    _last_now = now;

    // publish tf
    if(_rspub)
    {
        _rspub->publishTransforms(now, _tf_prefix);
    }

    // publish transform
    geometry_msgs::TransformStamped tf = tf2::eigenToTransform(T);
    std::string base_link;
    _model->getFloatingBaseLink(base_link);
    tf.child_frame_id = _tf_prefix_slash + base_link;
    tf.header.frame_id = _tf_prefix_slash + "world";
    tf.header.stamp = now;
    _base_pose_pub.publish(tf);

    // publish local twist
    Eigen::Vector6d v_local;
    v_local << T.linear().transpose()*v.head<3>(),
               T.linear().transpose()*v.tail<3>();

    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.stamp = now;
    twist_msg.header.frame_id = _tf_prefix_slash + base_link;

    tf::twistEigenToMsg(v_local, twist_msg.twist);
    _base_twist_pub.publish(twist_msg);

    // publish local raw twist
    Eigen::Vector6d raw_v_local;
    raw_v_local << T.linear().transpose()*raw_v.head<3>(),
                   T.linear().transpose()*raw_v.tail<3>();

    geometry_msgs::TwistStamped raw_twist_msg;
    raw_twist_msg.header.stamp = now;
    raw_twist_msg.header.frame_id = _tf_prefix_slash + base_link;

    tf::twistEigenToMsg(raw_v_local, raw_twist_msg.twist);
    _base_raw_twist_pub.publish(raw_twist_msg);

    // publish odom
    nav_msgs::Odometry odom_msg;
    odom_msg.header = tf.header;
    odom_msg.child_frame_id = tf.child_frame_id;
    tf::poseEigenToMsg(T, odom_msg.pose.pose);
    odom_msg.twist.twist = twist_msg.twist;

    // set covariance
    auto pose_cov = Eigen::Matrix6d::Map(odom_msg.pose.covariance.data());
    pose_cov.diagonal().head<3>().setConstant(_pose_lin_cov);
    pose_cov.diagonal().tail<3>().setConstant(_pose_rot_cov);

    auto vel_cov = Eigen::Matrix6d::Map(odom_msg.twist.covariance.data());
    vel_cov.diagonal().head<3>().setConstant(_vel_lin_cov);
    vel_cov.diagonal().tail<3>().setConstant(_vel_rot_cov);

    _base_odom_pub.publish(odom_msg);

    // publish tf
    tf::Transform t;
    tf::transformEigenToTF(T, t);
    _br.sendTransform(tf::StampedTransform(t, now, "world", base_link));
}

int main(int argc, char **argv)
{
    // init ros
    ros::init(argc, argv, "base_estimation_node");

    BaseEstimationNode node;

    ros::Rate rate(node.getRate());

    node.start();

    while(ros::ok())
    {
        node.run();

        rate.sleep();

    }

}


