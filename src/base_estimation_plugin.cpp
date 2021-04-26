#include "base_estimation_plugin.h"
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>

using namespace XBot;
using namespace std::string_literals;


bool BaseEstimationPlugin::on_initialize()
{
    // logger
    _logger = MatLogger2::MakeLogger("/tmp/ikbe_log");
    _logger->set_buffer_mode(VariableBuffer::Mode::circular_buffer);

    // print all
    setJournalLevel(Journal::Level::Low);

    // autostart
    setAutostart(true);

    // create model
    _model = ModelInterface::getModel(_robot->getConfigOptions());

    // check if floating base
    if(!_model->isFloatingBase())
    {
        jerror("model is fixed base");
        return false;
    }

    // set model to the robot state
    _robot->sense(false);
    _model->syncFrom(*_robot);

    // load problem
    auto ik_problem_yaml = getParamOrThrow<YAML::Node>("~ik_problem/content");

    // estimator options
    ikbe::BaseEstimation::Options est_opt;
    est_opt.dt = getPeriodSec();
    est_opt.log_enabled = getParamOr("~enable_log", false);
    est_opt.contact_attach_thr = getParamOr("~contact_attach_thr", 40.0);
    est_opt.contact_release_thr = getParamOr("~contact_release_thr", 10.0);

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
    // set world frame from ground truth
    else if(getParamOr("~world_from_gz", false))
    {
        auto lss = _robot->getDevices<Hal::LinkStateSensor>();

        std::string fb_name;
        _model->getFloatingBaseLink(fb_name);

        for(auto d : lss.get_device_vector())
        {
            if(d->getLinkName() == fb_name)
            {
                _gz = d;
            }
        }

        if(!_gz)
        {
            throw std::runtime_error(
                        fmt::format(
                            "link state sensor for link '{}' unavailable",
                            fb_name));
        }
    }

    // get contact properties

    // rolling contacts (ft name -> wheel name map)
    std::map<std::string, std::string> rolling_contacts;

    // get it from parameters
    getParam("~rolling_contacts", rolling_contacts);

    for(auto rc : rolling_contacts)
    {
        auto ft_name = rc.first;
        auto wh_name = rc.second;

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
        _est->addRollingContact(wh_name, ft);

        jinfo("adding rolling contact (ft: '{}', wheel: '{}')",
              ft_name,
              wh_name);
    }

    // surface contacts (including point contacts)
    std::map<std::string, std::string> surface_contacts;

    // get it from parameters
    getParam("~surface_contacts", surface_contacts);

    for(auto sc : surface_contacts)
    {
        // save force-torque name
        auto ft_name = sc.first;
        auto vertex_prefix = sc.second;

        // retrieve foot corner frames based on
        // the given prefix
        auto vertices = footFrames(vertex_prefix);

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
    ros::NodeHandle nh(getName());
    _ros = std::make_unique<RosSupport>(nh);
    _base_tf_pub = _ros->advertise<tf2_msgs::TFMessage>("/tf", 1);
    _base_pose_pub = _ros->advertise<geometry_msgs::PoseStamped>("/odometry/base_link/pose", 1);
    _base_twist_pub = _ros->advertise<geometry_msgs::TwistStamped>("/odometry/base_link/twist", 1);
    _base_raw_twist_pub = _ros->advertise<geometry_msgs::TwistStamped>("/odometry/base_link/raw_twist", 1);
    _contact_viz = std::make_shared<ikbe::contact_viz>("/odometry/contacts/weights", _ros.get());
    _contacts_state_pub = _ros->advertise<base_estimation::ContactsStatus>("/odometry/contacts/status",1);

    if(_gz)
    {
        _base_pose_gz_pub = _ros->advertise<geometry_msgs::PoseStamped>("/gazebo/base_link/pose", 1);
        _base_twist_gz_pub = _ros->advertise<geometry_msgs::TwistStamped>("/gazebo/base_link/twist", 1);
    }

    // advertise internal model state topic
    _model_state_msg.first.setZero(_model->getJointNum());
    _model_state_msg.second.setZero(_model->getJointNum());
    _model_state_pub = advertise<ModelState>("~model_state", _model_state_msg);


    // filter params
    double filter_param = 0.;
    if(getParam("~filter_omega", filter_param))
    {
        _est->setFilterOmega(filter_param);
    }
    if(getParam("~filter_damping", filter_param))
    {
        _est->setFilterDamping(filter_param);
    }

    _est->setFilterTs(getPeriodSec());

    return true;
}

void BaseEstimationPlugin::on_start()
{
    _robot->sense(false);
    _model->syncFrom(*_robot);

    if(_gz)
    {
        _model->setFloatingBaseState(_gz->getPose(),
                                     _gz->getTwist());
    }
    else if(_est->usesImu())
    {
        jinfo("resetting model from imu");
        _model->setFloatingBaseState(_imu);
    }

    _model->update();

    _est->reset();
}

void BaseEstimationPlugin::run()
{
    /* Update robot */
    _robot->sense(false);
    _model->syncFrom(*_robot);

    /* Update estimate */
    Eigen::Affine3d base_pose;
    Eigen::Vector6d base_vel, raw_base_vel;
    if(!_est->update(base_pose, base_vel, raw_base_vel))
    {
        jerror("unable to solve");
        return;
    }

    /* Publish contact markers in ROS */
    publishVertexWeights();
    publishContactStatus();

    /* Base state broadcast in ROS*/
    publishToROS(base_pose, base_vel, raw_base_vel);

    /* Model state broadcast */
    _model->getJointPosition(_model_state_msg.first);
    _model->getJointVelocity(_model_state_msg.second);
    _model_state_pub->publish(_model_state_msg);

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

void BaseEstimationPlugin::publishToROS(const Eigen::Affine3d& T, const Eigen::Vector6d& v,
                                        const Eigen::Vector6d& raw_v)
{

    // publish tf
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

    // publish geomsg
    geometry_msgs::PoseStamped Pmsg;
    convert(Tmsg, Pmsg);
    _base_pose_pub->publish(Pmsg);

    geometry_msgs::TwistStamped Vmsg;
    Vmsg.header = Tmsg.header;
    Vmsg.twist.linear.x = v[0];
    Vmsg.twist.linear.y = v[1];
    Vmsg.twist.linear.z = v[2];
    Vmsg.twist.angular.x = v[3];
    Vmsg.twist.angular.y = v[4];
    Vmsg.twist.angular.z = v[5];
    _base_twist_pub->publish(Vmsg);

    Vmsg.twist.linear.x = raw_v[0];
    Vmsg.twist.linear.y = raw_v[1];
    Vmsg.twist.linear.z = raw_v[2];
    Vmsg.twist.angular.x = raw_v[3];
    Vmsg.twist.angular.y = raw_v[4];
    Vmsg.twist.angular.z = raw_v[5];
    _base_raw_twist_pub->publish(Vmsg);

    // publish ground truth
    if(_gz)
    {
        tf::poseEigenToMsg(_gz->getPose(), Pmsg.pose);
        tf::twistEigenToMsg(_gz->getTwist(), Vmsg.twist);
        _base_pose_gz_pub->publish(Pmsg);
        _base_twist_gz_pub->publish(Vmsg);
    }

}

void BaseEstimationPlugin::publishContactStatus()
{
    base_estimation::ContactsStatus msg;
    ros::Time t = ros::Time::now();

    for(const auto& cinfo : _est->contact_info)
    {
        base_estimation::ContactStatus cs;

        cs.header.stamp = t;
        cs.header.frame_id = cinfo.name;
        cs.status = cinfo.contact_state;

        msg.contacts_status.push_back(cs);
    }

    _contacts_state_pub->publish(msg);
}

void BaseEstimationPlugin::publishVertexWeights()
{
    std::map<std::vector<std::string>, std::vector<double>> vwmap;

    for(const auto& cinfo : _est->contact_info)
    {
        vwmap[cinfo.vertex_frames] = cinfo.vertex_weights;
    }

    _contact_viz->publish(vwmap);
}

void BaseEstimationPlugin::convert(const geometry_msgs::TransformStamped& T,
                                   geometry_msgs::PoseStamped& P)
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

