#include "base_estimation_plugin.h"
#include <tf2_eigen/tf2_eigen.h>

using namespace XBot;
using namespace std::string_literals;


bool BaseEstimationPlugin::on_initialize()
{
    // print all
    setJournalLevel(Journal::Level::Low);

    // autostart
    setAutostart(true);

    // create model
    _model = ModelInterface::getModel(_robot->getConfigOptions());

    // set model to the robot state
    _robot->sense(false);
    _model->syncFrom(*_robot);

    // load problem
    auto ik_problem_yaml = getParamOrThrow<YAML::Node>("~ik_problem/content");

    // estimator options
    ikbe::BaseEstimation::Options est_opt;
    est_opt.log_enabled = getParamOr("~enable_log", false);

    // create estimator
    _est = std::make_unique<ikbe::BaseEstimation>(_model, ik_problem_yaml, est_opt);

    // use imu
    if(getParamOr<bool>("~use_imu", false))
    {
        if(!_robot->getImu().empty())
        {
            _imu = _robot->getImu().begin()->second;
            _est->addImu(_imu);
            jinfo("using imu '{}'", _imu->getSensorName());
        }
        else
        {
            throw std::runtime_error("no imu defined");
        }
    }

    // set world frame coincident to given link
    std::string world_frame_link = getParamOr("~world_frame_link", ""s);
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
    auto feet_prefix = getParamOrThrow<std::vector<std::string>>("~feet_prefix");
    auto ft_frames   = getParamOrThrow<std::vector<std::string>>("~force_torque_frames");
    if(feet_prefix.size() != ft_frames.size())
    {
        throw std::runtime_error("feet_prefix size is different from ft_frames size!");
    }

    // use ft sensors
    for(size_t i = 0; i < ft_frames.size(); i++)
    {
        std::string ft_name = ft_frames[i];

        auto ft = _robot->getForceTorque().at(ft_name);

        auto vertices = footFrames(feet_prefix[i]);

        _est->addFt(ft, vertices);

        jinfo("using ft '{}' with vertices: [{}]",
              ft->getSensorName(),
              fmt::join(vertices, ", "));
    }

    ros::NodeHandle nh(getName());
    _ros = std::make_unique<RosSupport>(nh);
    _base_tf_pub = _ros->advertise<tf2_msgs::TFMessage>("/tf", 1);
    _base_pose_pub = _ros->advertise<geometry_msgs::PoseStamped>("/odometry/base_link", 1);
    _base_twist_pub = _ros->advertise<geometry_msgs::TwistStamped>("/odometry/base_link_twist", 1);

    return true;
}

void BaseEstimationPlugin::on_start()
{
    _robot->sense(false);
    _model->syncFrom(*_robot);

    if(_est->usesImu())
    {
        jinfo("resetting model from imu");
        _model->setFloatingBaseState(_imu);
        _model->update();
    }

    _est->reset();
}

void BaseEstimationPlugin::run()
{
    /* Update robot */
    _robot->sense(false);
    _model->syncFrom(*_robot);

    /* Update estimate */
    Eigen::Affine3d base_pose;
    Eigen::Vector6d base_vel;
    if(!_est->update(base_pose, base_vel))
    {
        jerror("unable to solve");
    }

    /* Base Pose broadcast */
    publishToROS(base_pose, base_vel);
}

void BaseEstimationPlugin::on_stop()
{

}

std::vector<std::string> BaseEstimationPlugin::footFrames(const std::string& foot_prefix)
{
    std::vector<std::string> feet_tasks;
    auto ci = _est->ci();
    for(auto t : ci->getTaskList())
    {
        auto cart = std::dynamic_pointer_cast<Cartesian::CartesianTask>(ci->getTask(t));

        if(!cart)
        {
            continue;
        }

        if(t.length() >= foot_prefix.length() &&
                t.substr(0,foot_prefix.length()) == foot_prefix)
        {
            feet_tasks.push_back(t);
        }
    }

    return feet_tasks;
}

void BaseEstimationPlugin::publishToROS(const Eigen::Affine3d& T, const Eigen::Vector6d& v)
{
    tf2_msgs::TFMessage msg;

    geometry_msgs::TransformStamped Tmsg = tf2::eigenToTransform(T);
    std::string base_link;
    _model->getFloatingBaseLink(base_link);
    Tmsg.child_frame_id = base_link;
    Tmsg.header.frame_id = "odometry/world";
    ros::Time t;
    auto now = chrono::system_clock::now();
    auto now_ts = chrono::duration_chrono_to_timespec(now.time_since_epoch());
    t.sec = now_ts.tv_sec;
    t.nsec = now_ts.tv_nsec;
    Tmsg.header.stamp = t;

    msg.transforms.push_back(Tmsg);

    _base_tf_pub->publish(msg);
    geometry_msgs::PoseStamped Pmsg;
    convert(Tmsg, Pmsg);
    _base_pose_pub->publish(Pmsg);

    geometry_msgs::TwistStamped Vmsg;
    Vmsg.twist.linear.x = v[0];
    Vmsg.twist.linear.y = v[1];
    Vmsg.twist.linear.z = v[2];
    Vmsg.twist.angular.x = v[3];
    Vmsg.twist.angular.y = v[4];
    Vmsg.twist.angular.z = v[5];
    _base_twist_pub->publish(Vmsg);
}

void BaseEstimationPlugin::convert(const geometry_msgs::TransformStamped& T, geometry_msgs::PoseStamped& P)
{
    P.pose.position.x = T.transform.translation.x;
    P.pose.position.y = T.transform.translation.y;
    P.pose.position.z = T.transform.translation.z;

    P.pose.orientation.w = T.transform.rotation.w;
    P.pose.orientation.x = T.transform.rotation.x;
    P.pose.orientation.y = T.transform.rotation.y;
    P.pose.orientation.z = T.transform.rotation.z;

    P.header.frame_id = T.header.frame_id;
    P.header.stamp = T.header.stamp;
}

XBOT2_REGISTER_PLUGIN(BaseEstimationPlugin, base_estimation);

