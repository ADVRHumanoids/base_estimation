#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <xbot2_interface/robotinterface2.h>
#include <xbot2_interface/ros2/config_from_param.hpp>
#include <matlogger2/matlogger2.h>
#include <xbot2/journal/journal.h>

#include <base_estimation/base_estimation.h>
#include <base_estimation/msg/contacts_status.hpp>
#include <base_estimation/contact_viz.h>

#include "common.h"

using namespace std::string_literals;

class BaseEstimationNode : private XBot::Journal
{

public:

    BaseEstimationNode();

    double getRate() const;

    void start();

    bool run();

    rclcpp::Node::SharedPtr node();

private:

    rclcpp::Node::SharedPtr _node;

    XBot::RobotInterface::Ptr _robot;
    XBot::ModelInterface::Ptr _model;
    ikbe::BaseEstimation::UniquePtr _est;

    std::string _odom_frame; // odom frame name
    std::string _tf_prefix;
    bool _publish_tf; // whether to publish odom tf or not
    double _pose_lin_cov, _pose_rot_cov;
    double _vel_lin_cov, _vel_rot_cov;

    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr _base_tf_pub;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr _base_pose_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _base_twist_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _base_odom_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _base_raw_twist_pub;
    rclcpp::Publisher<base_estimation::msg::ContactsStatus>::SharedPtr _contacts_state_pub;

    rclcpp::Time _last_now;

    void publishToROS(const Eigen::Affine3d& T,
                      const Eigen::Vector6d& v,
                      const Eigen::Vector6d& raw_v);

};

BaseEstimationNode::BaseEstimationNode():
    XBot::Journal(XBot::Journal::no_publish,
                  "base_estimation_node")
{

    //
    _node = rclcpp::Node::make_shared("base_estimation_node");
    _last_now = _node->get_clock()->now();

    // get config options
    auto cfg = XBot::ConfigOptionsFromParams(_node);

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

    // load problem
    std::string ik_problem_str = _node->declare_parameter<std::string>("ik_problem");

    if(ik_problem_str.empty())
    {
        throw std::runtime_error("~ik_problem param missing");
    }

    auto ik_problem_yaml = YAML::Load(ik_problem_str);

    // estimator options
    ikbe::BaseEstimation::Options est_opt;
    est_opt.dt = 1./_node->declare_parameter("rate", 100.0);
    est_opt.log_enabled = _node->declare_parameter("enable_log", false);
    est_opt.contact_attach_thr = _node->declare_parameter("contact_attach_thr", 40.0);
    est_opt.contact_release_thr = _node->declare_parameter("contact_release_thr", 10.0);

    // create estimator
    _est = std::make_unique<ikbe::BaseEstimation>(_model,
                                                  ik_problem_yaml,
                                                  est_opt);

    // use imu
    if(_node->declare_parameter("use_imu", false))
    {
        if(!_robot->getImu().empty())
        {
            auto imu = _robot->getImu().begin()->second;
            _est->addImu(imu);
            jinfo("using imu '{}'", imu->getName());
        }
        else
        {
            throw std::runtime_error("no imu defined");
        }
    }

    // set world frame coincident to given link
    std::string world_from_tf = _node->declare_parameter("world_from_tf", ""s);
    if(world_from_tf != "")
    {
        tf2_ros::Buffer buffer(_node->get_clock());
        tf2_ros::TransformListener tl(buffer);

        std::string floating_base_link;
        _model->getFloatingBaseLink(floating_base_link);

        auto tf = buffer.lookupTransform(world_from_tf, floating_base_link, tf2::TimePoint(), 1s);

        Eigen::Affine3d fb_T_l = tf2::transformToEigen(tf.transform);

        _model->setFloatingBasePose(fb_T_l);
        _model->update();
    }

    // get contact properties

    // rolling contacts (ft name -> wheel name map)
    std::map<std::string, std::string> rolling_contacts;
    _node->declare_parameters("rolling_contacts", rolling_contacts);

    // z force override
    std::map<std::string, double> z_force_override;
    _node->declare_parameters("z_force_override", z_force_override);

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
            dummy_ft->setMeasurement(z_force_override.at(ft_name) * Eigen::Vector6d::Unit(2),
                                     XBot::wall_time::clock::now());  // useless timestamp
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
    _node->declare_parameters("surface_contacts", surface_contacts);

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
    _base_tf_pub = _node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 1);
    _base_pose_pub = _node->create_publisher<geometry_msgs::msg::TransformStamped>("base_link/pose", 1);
    _base_twist_pub = _node->create_publisher<geometry_msgs::msg::TwistStamped>("base_link/twist", 1);
    _base_raw_twist_pub = _node->create_publisher<geometry_msgs::msg::TwistStamped>("base_link/raw_twist", 1);
    _contacts_state_pub = _node->create_publisher<base_estimation::msg::ContactsStatus>("contacts/status", 1);
    _base_odom_pub = _node->create_publisher<nav_msgs::msg::Odometry>("base_link/odom", 1);

    // odom frame name
    _odom_frame = _node->declare_parameter("odom_frame", "odom");

    // Publish odom tf
    _publish_tf = _node->declare_parameter("publish_tf", true);

    // covariance
    _pose_lin_cov = _node->declare_parameter("pose_lin_cov", 1.0);
    _pose_rot_cov = _node->declare_parameter("pose_rot_cov", 1.0);
    _vel_lin_cov = _node->declare_parameter("vel_lin_cov", 1.0);
    _vel_rot_cov = _node->declare_parameter("vel_rot_cov", 1.0);

    // filter params
    double filter_param = 0.;

    if(_node->get_parameter("velocity_filter/omega", filter_param))
    {
        _est->setFilterOmega(filter_param);
    }

    if(_node->get_parameter("filter_damping", filter_param))
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

    if (_est->imu())
    {
        const auto imu = _est->imu();

        std::string base_link;
        _model->getFloatingBaseLink(base_link);

        // Transform from floating base to the physical IMU frame.
        const Eigen::Affine3d base_T_imu =
            _model->getPose(imu->getName(), base_link);

        // IMU gives world_R_imu. Convert it to world_R_base.
        Eigen::Affine3d world_T_base = Eigen::Affine3d::Identity();
        world_T_base.linear() =
            imu->getOrientation().toRotationMatrix() *
            base_T_imu.linear().transpose();

        // No global position is observable from this IMU.
        world_T_base.translation().setZero();
        world_T_base.linear().setZero();

        Eigen::Vector6d world_v_base = Eigen::Vector6d::Zero();
        _model->setFloatingBaseState(world_T_base, world_v_base);
    }

    _model->update();
    _est->reset();
}

bool BaseEstimationNode::run()
{
    // save fb pose
    auto Tfb = _model->getFloatingBasePose();
    auto vfb = _model->getFloatingBaseTwist();

    // update robot
    _robot->sense(false);
    _model->syncFrom(*_robot);

    _model->setFloatingBasePose(Tfb);
    _model->setFloatingBaseTwist(vfb);

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

rclcpp::Node::SharedPtr BaseEstimationNode::node()
{
    return _node;
}

void BaseEstimationNode::publishToROS(const Eigen::Affine3d& T,
                                      const Eigen::Vector6d& v,
                                      const Eigen::Vector6d& raw_v)
{
    // protect against duplicated tf warning
    auto now = _node->get_clock()->now();

    if(now == _last_now)
    {
        return;
    }

    _last_now = now;

    // publish transform
    geometry_msgs::msg::TransformStamped tf = tf2::eigenToTransform(T);
    std::string base_link;
    _model->getFloatingBaseLink(base_link);
    tf.child_frame_id = _tf_prefix + base_link;
    tf.header.frame_id = _tf_prefix + "odom";
    tf.header.stamp = now;
    _base_pose_pub->publish(tf);

    // publish local twist
    Eigen::Vector6d v_local;
    v_local << T.linear().transpose()*v.head<3>(),
               T.linear().transpose()*v.tail<3>();

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = now;
    twist_msg.header.frame_id = _tf_prefix + base_link;

    twist_msg.twist = tf2::toMsg(v_local);
    _base_twist_pub->publish(twist_msg);

    // publish local raw twist
    Eigen::Vector6d raw_v_local;
    raw_v_local << T.linear().transpose()*raw_v.head<3>(),
                   T.linear().transpose()*raw_v.tail<3>();

    geometry_msgs::msg::TwistStamped raw_twist_msg;
    raw_twist_msg.header.stamp = now;
    raw_twist_msg.header.frame_id = _tf_prefix + base_link;

    raw_twist_msg.twist = tf2::toMsg(raw_v_local);
    _base_raw_twist_pub->publish(raw_twist_msg);

    // publish odom
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header = tf.header;
    odom_msg.child_frame_id = tf.child_frame_id;
    odom_msg.pose.pose = tf2::toMsg(T);
    odom_msg.twist.twist = twist_msg.twist;

    // set covariance
    auto pose_cov = Eigen::Matrix6d::Map(odom_msg.pose.covariance.data());
    pose_cov.diagonal().head<3>().setConstant(_pose_lin_cov);
    pose_cov.diagonal().tail<3>().setConstant(_pose_rot_cov);

    auto vel_cov = Eigen::Matrix6d::Map(odom_msg.twist.covariance.data());
    vel_cov.diagonal().head<3>().setConstant(_vel_lin_cov);
    vel_cov.diagonal().tail<3>().setConstant(_vel_rot_cov);

    _base_odom_pub->publish(odom_msg);
 
    // publish odom frame
    if (_publish_tf) {
        tf.child_frame_id = base_link;
        tf.header.frame_id = _odom_frame;
        _base_pose_pub->publish(tf);

        tf2_msgs::msg::TFMessage tfmsg;
        tfmsg.transforms.push_back(tf);
        _base_tf_pub->publish(tfmsg);
    }
}

int main(int argc, char **argv)
{
    // init ros
    rclcpp::init(argc, argv);

    BaseEstimationNode node;

    rclcpp::Rate rate(node.getRate(), node.node()->get_clock());

    node.start();

    while(rclcpp::ok())
    {
        node.run();

        rate.sleep();
    }

}


