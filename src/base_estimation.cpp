﻿#include "base_estimation/base_estimation.h"

using namespace ikbe;
using namespace XBot;

template <typename T>
std::shared_ptr<T> task_as(Cartesian::TaskDescription::Ptr t)
{
    return std::dynamic_pointer_cast<T>(t);
}

ikbe::BaseEstimation::BaseEstimation(ModelInterface::Ptr model,
                                     YAML::Node contact_model_pb,
                                     Options opt):
    _model(model),
    _opt(opt),
    _alpha(model->getMass()*9.81)
{
    // create ci parameters
    auto ci_params = std::make_shared<Cartesian::Parameters>(opt.dt);
    ci_params->setLogEnabled(opt.log_enabled);

    // ci context
    auto ci_ctx = std::make_shared<Cartesian::Context>(ci_params, model);

    // ci ik problem
    Cartesian::ProblemDescription ik_problem(contact_model_pb, ci_ctx);

    // ci
    _ci = Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                          ik_problem,
                                                          ci_ctx);

    // get postural task to be kept in sync with robot
    // joint positions and velocities
    _postural = task_as<Cartesian::PosturalTask>(_ci->getTask("Postural"));

    // create a filter for the estimated twist
    _vel_filter = std::make_shared<XBot::Utils::SecondOrderFilter<Eigen::Vector6d>>();

    // logger
    _logger = XBot::MatLogger2::MakeLogger("/tmp/base_estimation_log");
    _logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
    _logger->create("fest_time", 1);
    _logger->create("ik_time", 1);
    _logger->create("vertex_opt_time", 1);
    _logger->create("update_time", 1);
}

BaseEstimation::Options BaseEstimation::getOptions() const
{
    return _opt;
}

ForceTorqueSensor::Ptr BaseEstimation::CreateDummyFtSensor(std::string name)
{
    auto ret = std::make_shared<ForceTorqueSensor>();
    ret->setWrench(Eigen::Vector6d::Zero(), 0.0);
    return ret;
}

Cartesian::CartesianInterfaceImpl::Ptr BaseEstimation::ci() const
{
    return _ci;
}

void BaseEstimation::addImu(ImuSensor::ConstPtr imu)
{
    _imu = imu;
    _imu_task = task_as<Cartesian::CartesianTask>(_ci->getTask(imu->getSensorName()));
    _imu_task->setActivationState(Cartesian::ActivationState::Enabled);

    // tbd: error check
}

bool BaseEstimation::usesImu() const
{
    return _imu != nullptr;
}

ImuSensor::ConstPtr BaseEstimation::imu() const
{
    return _imu;
}

ForceTorqueSensor::ConstPtr BaseEstimation::createVirtualFt(std::string link_name,
                                                            std::vector<int> dofs)
{
    using namespace XBot::Cartesian::Utils;

    // create force estimator if needed
    if(!_fest)
    {
        _fest = std::make_shared<ForceEstimation>(
                    _model,
                    // 1./getPeriodSec(),  // if using residuals,
                    ForceEstimation::DEFAULT_SVD_THRESHOLD);
    }

    // generate virtual ft
    auto ft = _fest->add_link(link_name, dofs);

    return ft;
}

void BaseEstimation::addSurfaceContact(std::vector<std::string> vertex_frames,
                                       ForceTorqueSensor::ConstPtr ft)
{
    ContactHandler ch;

    // ft pointer
    ch.ft = ft;

    // cartesian tasks for vertices
    std::transform(vertex_frames.begin(),
                   vertex_frames.end(),
                   std::back_inserter(ch.vertex_tasks),
                   [this](auto c)
    {
        return task_as<Cartesian::CartesianTask>(_ci->getTask(c));
    });

    // vertex force optimizer
    ch.vertex_opt = std::make_unique<VertexForceOptimizer>(ft->getSensorName(),
                                                           vertex_frames,
                                                           _model);
    ch.vertex_frames = vertex_frames;

    // contact estimator
    ch.contact_est = std::make_unique<ContactEstimation>(_opt.contact_release_thr,
                                                         _opt.contact_attach_thr);

    // push back contact info
    contact_info.emplace_back(ft->getSensorName(),
                              vertex_frames);

    _contact_handler.push_back(std::move(ch));

}

void BaseEstimation::addRollingContact(std::string wheel_name,
                                       ForceTorqueSensor::ConstPtr ft)
{
    ContactHandler ch;

    // ft pointer
    ch.ft = ft;

    // cartesio task for wheel rolling
    try
    {
        ch.vertex_tasks = { _ci->getTask("rolling_" + wheel_name) };
    }
    catch(std::out_of_range& e)
    {
        throw std::out_of_range("task 'rolling_" + wheel_name + "' undefined");
    }


    // vertex force optimizer's single vertex is located
    // at the ft frame
    ch.vertex_frames = { ft->getSensorName() };
    ch.vertex_opt = std::make_unique<VertexForceOptimizer>(ft->getSensorName(),
                                                           ch.vertex_frames,
                                                           _model);

    // push back contact info
    contact_info.emplace_back(ft->getSensorName(),
                              ch.vertex_frames);

    // contact estimator
    ch.contact_est = std::make_unique<ContactEstimation>(_opt.contact_release_thr,
                                                         _opt.contact_attach_thr);

    _contact_handler.push_back(std::move(ch));


}

bool BaseEstimation::update(Eigen::Affine3d& pose,
                            Eigen::Vector6d& vel,
                            Eigen::Vector6d& raw_vel)
{
    using clock_t = std::chrono::high_resolution_clock;
    auto total_tic = clock_t::now();

    // imu
    if(_imu)
    {
        // imu orientation
        Eigen::Matrix3d nwu_R_imu;
        _imu->getOrientation(nwu_R_imu);

        // imu ang velocity
        Eigen::Vector3d imu_vel_local;
        _imu->getAngularVelocity(imu_vel_local);
        Eigen::Vector3d imu_vel_world = nwu_R_imu * imu_vel_local;

        // set reference to imu task
        Eigen::Affine3d imu_ref;
        imu_ref.translation().setZero();
        imu_ref.linear() = nwu_R_imu;
        _imu_task->setPoseReference(imu_ref);

        Eigen::Vector6d imu_vel_ref;
        imu_vel_ref << 0., 0., 0., imu_vel_world;
        _imu_task->setVelocityReference(imu_vel_ref);
    }

    // ft and contacts


    auto tic = clock_t::now();
    if(_fest)
    {
        _fest->update();
    }
    _logger->add("fest_time", (clock_t::now() - tic).count()*1e-9);

    tic = clock_t::now();
    int i = 0;  // parallel iteration over contact_info
    for(auto& fthandler : _contact_handler)
    {
        auto& ft = *fthandler.ft;
        auto& vopt = *fthandler.vertex_opt;

        // get wrench from sensor
        Eigen::Vector6d wrench;
        ft.getWrench(wrench);

        // compute vertex forces
        _weights = vopt.compute(wrench);

        // normalize by robot weight
        _weights /= _alpha;


        // set weights to tasks
        for(size_t i = 0; i < fthandler.vertex_tasks.size(); ++i)
        {
            int size = fthandler.vertex_tasks[i]->getSize();
            fthandler.vertex_tasks[i]->setWeight(_weights[i]*Eigen::MatrixXd::Identity(size,size));
        }

        // update contact estimator and handle contact
        handle_contact_switch(fthandler);

        // save weights
        Eigen::VectorXd::Map(contact_info[i].vertex_weights.data(),
                             _weights.size()) = _weights;

        // save contact state
        contact_info[i].contact_state = fthandler.contact_est->getContactState();

        // save wrench
        contact_info[i].wrench = wrench;

        // increment contact_info index
        i++;

    }
    _logger->add("vertex_opt_time", (clock_t::now() - tic).count()*1e-9);

    // set joint velocities to postural task
    _model->getJointVelocity(_qdot);
    _postural->setReferenceVelocity(_qdot);

    // solve ik
    tic = clock_t::now();
    if(!_ci->update(0., _opt.dt))  // this updates model qdot
    {
        return false;
    }
    _logger->add("ik_time", (clock_t::now() - tic).count()*1e-9);

    // integrate solution
    _model->getJointPosition(_q);
    _model->getJointVelocity(_qdot);
    _q += _opt.dt * _qdot;
    _model->setJointPosition(_q);
    _model->update();  // note: update here?

    _model->getFloatingBasePose(pose);
    _model->getFloatingBaseTwist(raw_vel);

    // velocity filtering
    vel = _vel_filter->process(raw_vel);
    _model->setFloatingBaseTwist(vel);
    _model->update();

    _logger->add("update_time", (clock_t::now() - total_tic).count()*1e-9);

    return true;

}

void BaseEstimation::reset()
{
    _ci->reset(0.0);
}

bool BaseEstimation::reset(const std::string& task_name)
{
    auto task = _ci->getTask(task_name);
    if(task)
    {
        task->reset();
        return true;
    }
    return false;
}

BaseEstimation::Options::Options()
{
    dt = 1.0;
    log_enabled = false;
    contact_release_thr = 10.0;
    contact_attach_thr = 50.0;
}

void BaseEstimation::setFilterOmega(const double omega)
{
    _vel_filter->setOmega(omega);
}

void BaseEstimation::setFilterDamping(const double eps)
{
    _vel_filter->setDamping(eps);
}

void BaseEstimation::setFilterTs(const double ts)
{
    _vel_filter->setTimeStep(ts);
}

void BaseEstimation::handle_contact_switch(BaseEstimation::ContactHandler& fth)
{
    Eigen::Vector3d f;
    fth.ft->getForce(f);

    // note: normal always local z-axis?
    double f_n = f.z();

    // if contact is created..
    if(fth.contact_est->update(f_n) ==
            ContactEstimation::Event::Attached)
    {
        // reset reference for all vertex frames
        for(auto t : fth.vertex_tasks)
        {
            t->reset();
        }
    }
}

BaseEstimation::ContactInformation::ContactInformation(
        std::string _name,
        std::vector<std::string> _vertex_frames):
    name(_name),
    vertex_frames(_vertex_frames),
    vertex_weights(_vertex_frames.size(), 0.0),
    contact_state(true)
{
    wrench.setZero();
}
