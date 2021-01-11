#include "base_estimation.h"
#include <tf2_eigen/tf2_eigen.h>
#include <OpenSoT/solvers/BackEndFactory.h>


using namespace XBot;


ContactForceOptimization::ContactForceOptimization(const std::string& sensor_frame,
                                                   const std::vector<std::string>& corner_frames,
                                                   ModelInterface::ConstPtr model):
    _model(model)
{
    _solver = OpenSoT::solvers::BackEndFactory(OpenSoT::solvers::solver_back_ends::qpOASES, 4, 0,
                                               OpenSoT::HessianType::HST_POSDEF, 1e6);

    _A.setOnes(3,4);

    Eigen::Affine3d T;
    int col = 0;
    for(auto frame : corner_frames)
    {
        _model->getPose(frame, sensor_frame, T);
        _A(1,col) = T.translation()[0];
        _A(2,col) = T.translation()[1];
        col += 1;
    }

    _H = _A.transpose()*_A;

    _b.setZero(3);

    _lb.setZero(3); _ub.setZero(3);
    _ub<<1e6, 1e6, 1e6, 1e6;

    _g.setZero(4);

    _solver->initProblem(_H, _g, Eigen::MatrixXd(0,0), Eigen::VectorXd(0), Eigen::VectorXd(0), _lb, _ub);
}

const Eigen::VectorXd& ContactForceOptimization::compute(const double Fz, const double Mx, const double My)
{
    _b[0] = Fz;
    _b[1] = -My;
    _b[2] = Mx;

    _g = -_A.transpose()*_b;

    _solver->updateTask(_H, _g);
    _solver->solve();

    return _solver->getSolution();
}

bool BaseEstimation::on_initialize()
{
    setAutostart(true);

    _model = ModelInterface::getModel(_robot->getConfigOptions());

    Eigen::VectorXd q_home;
    _model->getRobotState("home", q_home);
    _model->setJointPosition(q_home);
    _model->update();


    _use_imu = getParamOr<bool>("~use_imu", false);
    if(_use_imu)
    {
        if(!_robot->getImu().empty())
        {
            _imu = _robot->getImu().begin()->second;
        }
        else
        {
            throw std::runtime_error("No IMU on robot!");
        }
    }

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

    if(_imu)
    {
        _imu_task = std::dynamic_pointer_cast<Cartesian::CartesianTask>(_ci->getTask("imu_link"));
    }

    ros::NodeHandle nh(getName());
    _ros = std::make_unique<RosSupport>(nh);
    _base_tf_pub = _ros->advertise<tf2_msgs::TFMessage>("/tf", 1);
    _base_pose_pub = _ros->advertise<geometry_msgs::PoseStamped>("/odometry/base_link", 1);


    return true;
}

void BaseEstimation::on_start()
{
    if(_imu)
    {
        _model->setFloatingBaseState(_imu);
        _model->update();
        _ci->reset(0.);
    }
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

    /* IMU */
    if(_imu)
    {
        Eigen::Matrix3d nwu_R_imu;
        _imu->getOrientation(nwu_R_imu);
        _w_R_imu = nwu_R_imu;

        Eigen::Vector3d tmp_vel;
        _imu->getAngularVelocity(tmp_vel);
        _v_imu = _w_R_imu*tmp_vel;

        Eigen::Affine3d imu_ref;
        imu_ref.translation().setZero();
        imu_ref.linear() = _w_R_imu;
        _imu_task->setPoseReference(imu_ref);
        Eigen::Vector6d imu_vel_ref; imu_vel_ref<<0.,0.,0.,_v_imu;
        _imu_task->setVelocityReference(imu_vel_ref);
    }

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
    publishToROS(world_T_base);
}

void BaseEstimation::publishToROS(const Eigen::Affine3d& T)
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
}

void BaseEstimation::convert(const geometry_msgs::TransformStamped& T, geometry_msgs::PoseStamped& P)
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

XBOT2_REGISTER_PLUGIN(BaseEstimation, base_estimation);

