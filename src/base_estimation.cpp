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
    // create ci
    auto ci_params = std::make_shared<Cartesian::Parameters>(opt.dt);
    ci_params->setLogEnabled(opt.log_enabled);

    auto ci_ctx = std::make_shared<Cartesian::Context>(ci_params, model);

    Cartesian::ProblemDescription ik_problem(contact_model_pb, ci_ctx);

    _ci = Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                          ik_problem,
                                                          ci_ctx);

    // get postural
    _postural = task_as<Cartesian::PosturalTask>(_ci->getTask("Postural"));

    _vel_filter = std::make_shared<XBot::Utils::SecondOrderFilter<Eigen::Vector6d> > ();
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

    // optimizer
    fth.vertex_opt = std::make_unique<VertexForceOptimizer>(ft->getSensorName(),
                                                            contact_points,
                                                            _model);
    fth.vertex_frames = contact_points;

    _ft_handler.push_back(std::move(fth));

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

    // ft
    _map_vertex_frames_weights.clear();
    for(auto& item : _ft_handler)
    {
        auto& ft = *item.ft;
        auto& cf = *item.vertex_opt;

        Eigen::Vector6d wrench;
        ft.getWrench(wrench);

        _weights = cf.compute(wrench);
        _weights /= _alpha;

        _map_vertex_frames_weights[item.vertex_frames] = _weights;

        for(size_t i = 0; i < item.vertex_tasks.size(); ++i)
        {
            int size = item.vertex_tasks[i]->getSize();
            item.vertex_tasks[i]->setWeight(_weights[i]*Eigen::MatrixXd::Identity(size,size));
        }
    }

    /* Set joint velocities to postural task */
    _model->getJointVelocity(_qdot);
    _postural->setReferenceVelocity(_qdot);

    /* Solve IK */
    if(!_ci->update(0., _opt.dt))  // this updates model qdot
    {
        return false;
    }

    /* Integrate solution */
    _model->getJointPosition(_q);
    _model->getJointVelocity(_qdot);
    _q += _opt.dt * _qdot;
    _model->setJointPosition(_q);
    _model->update();

    _model->getFloatingBasePose(pose);
    _model->getFloatingBaseTwist(raw_vel);

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
