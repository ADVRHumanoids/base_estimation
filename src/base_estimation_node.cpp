#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>

#include <RobotInterfaceROS/ConfigFromParam.h>
#include <XBotInterface/RobotInterface.h>
#include <matlogger2/matlogger2.h>
#include <xbot2/journal/journal.h>

#include <base_estimation/base_estimation.h>
#include <base_estimation/ContactsStatus.h>
#include <base_estimation/ContactsWrench.h>
#include <base_estimation/contact_viz.h>
#include <std_msgs/Bool.h>

#include "common.h"

#include <base_estimation/contact_estimation.h>

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
    ros::NodeHandle _nhpr;

    XBot::RobotInterface::Ptr _robot;
    XBot::ModelInterface::Ptr _model;
    ikbe::BaseEstimation::UniquePtr _est;

    std::string _odom_frame;

    ros::Publisher _base_tf_pub;
    ros::Publisher _base_pose_pub;
    ros::Publisher _base_twist_pub;
    ros::Publisher _base_raw_twist_pub;
    ros::Publisher _contacts_state_pub;
    ros::Publisher _haptic_contacts_state_pub;
    ros::Publisher _wrenches_pub;

    void publishToROS(const Eigen::Affine3d& T,
                      const Eigen::Vector6d& v,
                      const Eigen::Vector6d& raw_v,
                      const std::vector<bool>& contact_flags,
                      const std::vector<bool>& haptic_contact_flags,
                      const std::vector<Eigen::Vector6d>& contact_wrenches);

    bool _publish_tftopic;
    int _contacts_number;   // the total number of contacts

};

BaseEstimationNode::BaseEstimationNode():
    XBot::Journal(XBot::Journal::no_publish, "base_estimation_node"),
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
    est_opt.estimate_contacts = _nhpr.param("estimate_contacts", false);

    // publish or not to /tf topic
    _publish_tftopic = _nhpr.param("publish_tftopic", false);

    // create estimator
    if (est_opt.estimate_contacts) {
        std::cout << "Contacts to be estimated." << std::endl;
    }
    else {
        std::cout << "Contacts are preplanned." << std::endl;
    }
    _est = std::make_unique<ikbe::BaseEstimation>(_model, ik_problem_yaml, _nhpr, est_opt);

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
    std::string world_frame_link = _nhpr.param("world_frame_link", ""s);
    std::cout << "World frame link: " << world_frame_link << std::endl;
    if(world_frame_link == "")
    {
        // initialize base pose
        const std::string basePoseTopic = "/xbotcore/link_state/pelvis/pose";
        boost::shared_ptr<geometry_msgs::PoseStamped const> msgBasePosePtr;             // ros messages
        msgBasePosePtr = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(basePoseTopic, _nhpr, ros::Duration(3));

        if (msgBasePosePtr != NULL) {                           // check if xbotcore pose msg is available (simulation)
            Eigen::Affine3d fb_T_l;
            tf::poseMsgToEigen(msgBasePosePtr->pose, fb_T_l);
            _model->setFloatingBasePose(fb_T_l);
            jinfo("Initialized base pose from xbotcore topic");
        }
        else {                                                  // otherwise initialize with a default value
            Eigen::Affine3d fb_T_l;
            fb_T_l.translation() = Eigen::Vector3d(-0.064, -0.004, 0.719);
            fb_T_l.linear() << 0.999948,    0.0101555,  0.00109614,
                              -0.0101556,  0.999948,   8.00347e-06,
                              -0.001096,   -1.9135e-05,    0.999999;
            _model->setFloatingBasePose(fb_T_l);
            jinfo("Did not get message from xbotcore. Initialized base pose from default values");
        }
    } else if(world_frame_link != "") {
        // previous implementation
        Eigen::Affine3d fb_T_l;
        std::string floating_base_link;
        _model->getFloatingBaseLink(floating_base_link);
        if(!_model->getPose(world_frame_link, floating_base_link, fb_T_l))
        {
          throw std::runtime_error("world frame link '" + world_frame_link + "' is undefined");
        }
        jinfo("using link '{}' as world frame", world_frame_link);
        _model->setFloatingBasePose(fb_T_l.inverse());
    }
    _model->update();

    // get contact properties
    std::map<std::string, std::string> rolling_contacts;            // rolling contacts (ft name -> wheel name map)
    _nhpr.getParam("rolling_contacts", rolling_contacts);

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
            ft = ft_map.at(ft_name);
        else
            ft = _est->createVirtualFt(ft_name, {0, 1, 2});

        // create contact
        _est->addRollingContact(wh_name, ft);

        jinfo("adding rolling contact (ft: '{}', wheel: '{}')", ft_name, wh_name);
    }

    // surface contacts (including point contacts)
    std::map<std::string, std::string> surface_contacts, arm_surface_contacts;
    _nhpr.getParam("surface_contacts", surface_contacts);                              // first get feet
    _nhpr.getParam("arm_surface_contacts", arm_surface_contacts);                         // then arms

    std::vector<std::map<std::string, std::string>> mapsVector = {surface_contacts, arm_surface_contacts};

    for (auto contact_species : mapsVector) {
        for(auto sc : contact_species)
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
                ft = ft_map.at(ft_name);
            else
                ft = _est->createVirtualFt(ft_name, {0, 1, 2});

            // create contact
            if (!vertices.empty())
                _est->addSurfaceContact(vertices, ft);

            jinfo("adding surface contact '{}' with vertices: [{}]",
                    ft_name,
                    fmt::join(vertices, ", "));
        }
    }

    // set number of total contact
    _contacts_number = rolling_contacts.size() + surface_contacts.size() + arm_surface_contacts.size();

    // publishers
    if (_publish_tftopic)
        _base_tf_pub = _nhpr.advertise<tf2_msgs::TFMessage>("/tf", 1);
    _base_pose_pub = _nhpr.advertise<geometry_msgs::TransformStamped>("base_link/pose", 1);
    _base_twist_pub = _nhpr.advertise<geometry_msgs::TwistStamped>("base_link/twist", 1);
    _base_raw_twist_pub = _nhpr.advertise<geometry_msgs::TwistStamped>("base_link/raw_twist", 1);
    _contacts_state_pub = _nhpr.advertise<base_estimation::ContactsStatus>("contacts/status", 1);
    _wrenches_pub = _nhpr.advertise<base_estimation::ContactsWrench>("contacts/wrench", 1);
    _haptic_contacts_state_pub = _nhpr.advertise<base_estimation::ContactsStatus>("contacts/haptic_status", 1);

    // odom frame name
    _odom_frame = _nhpr.param("odom_frame", "odometry/world"s);

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
    _robot->sense(true);
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

    // contact status and contact wrenches
    std::vector<Eigen::Vector6d> wrenches(_contacts_number);
    std::vector<bool> contacts(_contacts_number);
    std::vector<bool> haptic_contacts(_contacts_number);
    for (int i = 0; i < contacts.size(); i++) {
        contacts[i] = _est->contact_info[i].contact_state;
        haptic_contacts[i] = _est->contact_info[i].contact_haptic_state;
        wrenches[i] = _est->contact_info[i].wrench;
    }

    // base state broadcast in ROS
    publishToROS(base_pose, base_vel, raw_base_vel, contacts, haptic_contacts, wrenches);

    return true;
}

void BaseEstimationNode::publishToROS(const Eigen::Affine3d& T, const Eigen::Vector6d& v, const Eigen::Vector6d& raw_v,
                                      const std::vector<bool>& contact_flags,
                                      const std::vector<bool>& haptic_contact_flags,
                                      const std::vector<Eigen::Vector6d>& contact_wrenches)
{
    // publish tf
    geometry_msgs::TransformStamped tf = tf2::eigenToTransform(T);
    std::string base_link;
    _model->getFloatingBaseLink(base_link);
    tf.child_frame_id = base_link;
    tf.header.frame_id = "odometry/world";
    tf.header.stamp = ros::Time::now();

    if (_publish_tftopic) {
        tf2_msgs::TFMessage msg;
        msg.transforms.push_back(tf);
        _base_tf_pub.publish(msg);
    }

    // publish geomsg
    _base_pose_pub.publish(tf);

    geometry_msgs::TwistStamped Vmsg;
    Vmsg.header = tf.header;
    tf::twistEigenToMsg(v, Vmsg.twist);
    _base_twist_pub.publish(Vmsg);

    tf::twistEigenToMsg(raw_v, Vmsg.twist);
    _base_raw_twist_pub.publish(Vmsg);

    // publish contact status
    base_estimation::ContactsStatus contactsMsg;
    base_estimation::ContactsStatus hapticContactsMsg;
    base_estimation::ContactStatus singleContactMsg;
    for (int i = 0; i < contact_flags.size(); i++) {            // fill message
        singleContactMsg.status = contact_flags[i];
        singleContactMsg.header = tf.header;
        contactsMsg.contacts_status.emplace_back(singleContactMsg);

        // hapic contact msg
        singleContactMsg.status = haptic_contact_flags[i];
        hapticContactsMsg.contacts_status.emplace_back(singleContactMsg);
    }
    _contacts_state_pub.publish(contactsMsg);
    _haptic_contacts_state_pub.publish(hapticContactsMsg);   // publish contact haptic status

    // publish contact wrench
    geometry_msgs::WrenchStamped singleWrenchMsg;
    base_estimation::ContactsWrench wrenchMsg;
    for (int i = 0; i < contact_wrenches.size(); i++) {            // fill message
        tf::wrenchEigenToMsg(contact_wrenches[i], singleWrenchMsg.wrench);
        singleWrenchMsg.header = tf.header;
        wrenchMsg.contacts_wrench.emplace_back(singleWrenchMsg);
        auto forceNorm = sqrt(singleWrenchMsg.wrench.force.x*singleWrenchMsg.wrench.force.x + singleWrenchMsg.wrench.force.y*singleWrenchMsg.wrench.force.y +
                              singleWrenchMsg.wrench.force.z*singleWrenchMsg.wrench.force.z);
        wrenchMsg.force_norm.emplace_back(forceNorm);
    }
    _wrenches_pub.publish(wrenchMsg);
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
        ros::spinOnce();    // spin for the preplanned contacts subscriber
        rate.sleep();

    }

}


