#include "base_estimation.h"
#include <tf2_eigen/tf2_eigen.h>



using namespace XBot;

bool BaseEstimation::on_initialize()
{
    setAutostart(true);

    _model = ModelInterface::getModel(_robot->getConfigOptions());

    Eigen::VectorXd q_home;
    _model->getRobotState("home", q_home);
    _model->setJointPosition(q_home);
    _model->update();

    auto ci_params = std::make_shared<Cartesian::Parameters>(getPeriodSec());
    ci_params->setLogEnabled(getParamOr("~enable_log", false));
    auto ci_ctx = std::make_shared<Cartesian::Context>(ci_params, _model);

    auto ik_problem_yaml = getParamOrThrow<YAML::Node>("~ik_problem/content");
    Cartesian::ProblemDescription ik_problem(ik_problem_yaml, ci_ctx);

    std::string world_frame_link = getParamOr<std::string>("~world_frame_link", "");
    if(world_frame_link != "")
    {
        Eigen::Affine3d fb_T_l;
        std::string floating_base_link;
        _model->getFloatingBaseLink(floating_base_link);
        if(!_model->getPose(world_frame_link, floating_base_link, fb_T_l))
        {
            throw std::runtime_error("World frame link '" + world_frame_link + "' is undefined");
        }

        _model->setFloatingBasePose(fb_T_l.inverse());
        _model->update();
    }

    _ci = Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot", ik_problem, ci_ctx);

    _postural = std::dynamic_pointer_cast<Cartesian::PosturalTask>(_ci->getTask("Postural"));

    ros::NodeHandle nh(getName());
    _ros = std::make_unique<RosSupport>(nh);
    _base_transform_pub = _ros->advertise<tf2_msgs::TFMessage>("/tf", 1);

    return true;
}

void BaseEstimation::on_start()
{

}

void BaseEstimation::on_stop()
{

}

void BaseEstimation::run()
{
    /* Update robot */
    _robot->sense(false);
    _model->syncFrom(*_robot);
    _model->getJointVelocity(_qdot);

    /* Set joint velocities to postural task */
    _postural->setReferenceVelocity(_qdot);

    /* Solve IK */
    if(!_ci->update(0., getPeriodSec()))
    {
        jerror("unable to solve \n");
        return;
    }

    /* Integrate solution */
    _model->getJointPosition(_q);
    _model->getJointVelocity(_qdot);
    _q += getPeriodSec() * _qdot;
    _model->setJointPosition(_q);
    _model->update();

    Eigen::Affine3d world_T_base;
    _model->getFloatingBasePose(world_T_base);

    /* Base Pose broadcast */
    publishTf(world_T_base);
}

void BaseEstimation::publishTf(const Eigen::Affine3d& T)
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

    _base_transform_pub->publish(msg);
}

XBOT2_REGISTER_PLUGIN(BaseEstimation, base_estimation);

