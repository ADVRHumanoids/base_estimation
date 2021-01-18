#include "cartesio_rt.h"

using namespace XBot;
using namespace XBot::Cartesian;

bool CartesioRt::on_initialize()
{
    setJournalLevel(Journal::Level::Low);

    _nh = std::make_unique<ros::NodeHandle>();

    /* Get ik problem from ros param */
    auto problem_param = getParamOr<std::string>("~problem_param",
                                                 "cartesian/problem_description");
    std::string ik_str;
    if(getParam("~problem_description/content", ik_str))
    {

    }
    else if(!_nh->getParam(problem_param, ik_str))
    {
        jerror("ros param '{}' not found", problem_param);
        return false;
    }

    auto ik_yaml = YAML::Load(ik_str);

    /* Create model and ci for rt loop */
    _rt_model = ModelInterface::getModel(_robot->getConfigOptions());

    double joint_impedance_gain = getParamOr<double>("~joint_impedance_scaling", 0.1);

    Eigen::VectorXd act_joint_stiffness, act_joint_damping;
    _robot->getStiffness(act_joint_stiffness);
    _robot->getDamping(act_joint_damping);
    Eigen::VectorXd joint_stiffness(_rt_model->getJointNum()), joint_damping(_rt_model->getJointNum());
    joint_stiffness<<0.,0.,0.,0.,0.,0.,joint_impedance_gain*act_joint_stiffness;
    joint_damping<<0.,0.,0.,0.,0.,0.,std::sqrt(joint_impedance_gain)*act_joint_damping;
    _rt_model->setStiffness(joint_stiffness);
    _rt_model->setDamping(joint_damping);
    _rt_model->update();

    auto rt_ctx = std::make_shared<Cartesian::Context>(
                std::make_shared<Parameters>(getPeriodSec()),
                _rt_model);

    rt_ctx->params()->setLogEnabled(getParamOr("~enable_log", false));

    ProblemDescription ik_problem(ik_yaml, rt_ctx);

    auto impl_name = getParamOr<std::string>("~solver", "OpenSot");

    _rt_ci = CartesianInterfaceImpl::MakeInstance(impl_name, ik_problem, rt_ctx);
    _rt_ci->enableOtg(rt_ctx->params()->getControlPeriod());
    _rt_ci->update(0, 0);

    /* Create model and ci for nrt loop */
    auto nrt_model = ModelInterface::getModel(_robot->getConfigOptions());

    auto nrt_ctx = std::make_shared<Cartesian::Context>(
                std::make_shared<Parameters>(*_rt_ci->getContext()->params()),
                nrt_model);

    _nrt_ci = std::make_shared<LockfreeBufferImpl>(_rt_ci.get(), nrt_ctx);
    _nrt_ci->pushState(_rt_ci.get(), _rt_model.get());
    _nrt_ci->updateState();
    auto nrt_ci = _nrt_ci;

    /*  Create ros api server */
    RosServerClass::Options opt;
    opt.tf_prefix = getParamOr<std::string>("~tf_prefix", "ci");
    opt.ros_namespace = getParamOr<std::string>("~ros_ns", "cartesian");
    auto ros_srv = std::make_shared<RosServerClass>(_nrt_ci, opt);

    /* Initialization */
    _rt_active = false;
    auto rt_active_ptr = &_rt_active;

    _nrt_exit = false;
    auto nrt_exit_ptr = &_nrt_exit;

    _qdot = _q.setZero(_rt_model->getJointNum());
    _robot->getPositionReference(_qmap);

    /* Spawn thread */
    _nrt_th = std::make_unique<thread>(
                [rt_active_ptr, nrt_exit_ptr, nrt_ci, ros_srv]()
    {
        this_thread::set_name("cartesio_nrt");

        while(!*nrt_exit_ptr)
        {
            this_thread::sleep_for(10ms);

            if(!*rt_active_ptr) continue;

            nrt_ci->updateState();
            ros_srv->run();

        }

    });

    /* Set robot control mode */
    _robot->setControlMode(ControlMode::Position() + ControlMode::Effort());

    /* Feedback */
    _enable_feedback = getParamOr("~enable_feedback", false);

    if(_enable_feedback)
    {
        jinfo("running with feedback enabled \n");
    }

    /* Model state topic (optional) */
    _model_state_recv = false;
    if(getParamOr("~model_state_from_topic", false))
    {
        _model_state_sub = subscribe("~model_state",
                                     &CartesioRt::on_model_state_recv,
                                     this,
                                     1);
    }

    /* Ground truth */
    if(getParamOr("~use_ground_truth", false))
    {
        auto lss = _robot->getDevices<Hal::LinkStateSensor>();

        std::string fb_name;
        _rt_model->getFloatingBaseLink(fb_name);

        for(auto d : lss.get_device_vector())
        {
            if(d->getLinkName() == fb_name)
            {
                _fb_truth = d;
            }
        }

        if(!_fb_truth)
        {
            throw std::runtime_error(
                        fmt::format(
                            "link state sensor for link '{}' unavailable",
                            fb_name));
        }
    }

    return true;
}

void CartesioRt::on_start()
{

}

void CartesioRt::starting()
{
    // wait first model state from topic
    if(_model_state_sub)
    {
        // listen to messages
        _model_state_sub->run();

        // nothing.. will retry
        if(!_model_state_recv)
        {
            XBOT2_JINFOP_EVERY(HIGH, (*this), 1s,
                               "waiting for model state..");
            return;
        }

        // ok, model updated by sub callback

    }
    else
    {
        // align model to current position reference
        _robot->sense(false);
        _robot->getPositionReference(_qmap);
        _rt_model->setJointPosition(_qmap);
        _rt_model->update();

        // optional ground truth option
        if(_fb_truth)
        {
            _rt_model->setFloatingBaseState(_fb_truth->getPose(),
                                            _fb_truth->getTwist());
            _rt_model->update();
        }
    }

    // we use a fake time, and integrate it by the expected dt
    _fake_time = 0;

    // reset ci
    _rt_ci->reset(_fake_time);

    // signal nrt thread that rt is active
    _rt_active = true;

    // transit to run
    start_completed();

}

void CartesioRt::run()
{
    /* Receive commands from nrt */
    _nrt_ci->callAvailable(_rt_ci.get());

    /* Update robot */
    if(_enable_feedback)
    {
        _robot->sense(false);

        // update model state from topic or robot
        if(_model_state_sub)
        {
            _model_state_sub->run();
        }
        else
        {
            _rt_model->syncFrom(*_robot);

            // optional ground truth option
            if(_fb_truth)
            {
                _rt_model->setFloatingBaseState(_fb_truth->getPose(),
                                                _fb_truth->getTwist());
                _rt_model->update();
            }
        }

        // tbd: optional ground truth option

    }

    /* Solve IK */
    if(!_rt_ci->update(_fake_time, getPeriodSec()))
    {
        jerror("unable to solve");
        return;
    }

    /* Integrate solution */
    if(!_enable_feedback)
    {
        _rt_model->getJointPosition(_q);
        _rt_model->getJointVelocity(_qdot);
        _q += getPeriodSec() * _qdot;
        _rt_model->setJointPosition(_q);
        _rt_model->update();
    }


    _fake_time += getPeriodSec();

    /* Send state to nrt */
    _nrt_ci->pushState(_rt_ci.get(), _rt_model.get());

    /* Move robot */
    if(_enable_feedback)
    {
        _robot->setReferenceFrom(*_rt_model, Sync::Effort);
    }
    else
    {
        _robot->setReferenceFrom(*_rt_model);
    }

    _robot->move();
}

void CartesioRt::stopping()
{
    _rt_active = false;
    stop_completed();
}

void CartesioRt::on_abort()
{
    _rt_active = false;
    _nrt_exit = true;
}

void CartesioRt::on_close()
{
    _nrt_exit = true;
    jinfo("joining with nrt thread.. \n");
    if(_nrt_th) _nrt_th->join();
}

void CartesioRt::on_model_state_recv(const CartesioRt::ModelState &msg)
{
    _rt_model->setJointPosition(msg.first);
    _rt_model->setJointVelocity(msg.second);
    _rt_model->update();

    _model_state_recv = true;
}

XBOT2_REGISTER_PLUGIN(CartesioRt,
                      cartesio_plugin);
