#include "base_estimation/base_estimation.h"

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

void BaseEstimation::addFt(ForceTorqueSensor::ConstPtr ft,
                           std::vector<std::string> contact_points)
{
    FtHandler fth;

    // ft pointer
    fth.ft = ft;

    // cartesian tasks for vertices
    std::transform(contact_points.begin(),
                   contact_points.end(),
                   std::back_inserter(fth.vertex_tasks),
                   [this](auto c)
    {
        return task_as<Cartesian::CartesianTask>(_ci->getTask(c));
    });

    // vertex force optimizer
    fth.vertex_opt = std::make_unique<VertexForceOptimizer>(ft->getSensorName(),
                                                            contact_points,
                                                            _model);
    fth.vertex_frames = contact_points;

    // contact estimator
    fth.contact_est = std::make_unique<ContactEstimation>(_opt.contact_release_thr,
                                                          _opt.contact_attach_thr);

    _ft_handler.push_back(std::move(fth));

    // push back contact info
    contact_info.emplace_back(ft->getSensorName(),
                              contact_points);


}

void BaseEstimation::addVirtualFt(std::string link_name,
                                  std::vector<int> dofs,
                                  std::vector<std::string> contact_points)
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

    // add it to the estimator
    addFt(ft, contact_points);

}

bool BaseEstimation::update(Eigen::Affine3d& pose,
                            Eigen::Vector6d& vel,
                            Eigen::Vector6d& raw_vel)
{
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
    if(_fest)
    {
        _fest->update();
    }

    int i = 0;  // parallel iteration over contact_info
    for(auto& fthandler : _ft_handler)
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

    // set joint velocities to postural task
    _model->getJointVelocity(_qdot);
    _postural->setReferenceVelocity(_qdot);

    // solve ik
    if(!_ci->update(0., _opt.dt))  // this updates model qdot
    {
        return false;
    }

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

void BaseEstimation::handle_contact_switch(BaseEstimation::FtHandler& fth)
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
        for(auto frame : fth.vertex_frames)
        {
            reset(frame);
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
