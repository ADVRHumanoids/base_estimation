#ifndef __IK_BASE_ESTIMATION_H__
#define __IK_BASE_ESTIMATION_H__

#include "vertex_force_optimizer.h"

#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/problem/Postural.h>

namespace ikbe
{

class BaseEstimation
{

public:

    typedef std::unique_ptr<BaseEstimation> UniquePtr;

    struct Options
    {
        double dt;
        bool log_enabled;

        Options();
    };

    BaseEstimation(XBot::ModelInterface::Ptr model,
                   YAML::Node est_model_pb,
                   Options opt = Options()
                   );

    XBot::Cartesian::CartesianInterfaceImpl::Ptr ci() const;

    void addImu(XBot::ImuSensor::ConstPtr imu);

    bool usesImu() const;

    void addFt(XBot::ForceTorqueSensor::ConstPtr ft,
               std::vector<std::string> contact_points);

    bool update(Eigen::Affine3d& pose,
                Eigen::Vector6d& vel);

    void reset();

private:

    Options _opt;

    Eigen::VectorXd _q, _qdot;
    XBot::ModelInterface::Ptr _model;
    XBot::Cartesian::CartesianInterfaceImpl::Ptr _ci;
    XBot::Cartesian::PosturalTask::Ptr _postural;

    XBot::Cartesian::CartesianTask::Ptr _imu_task;
    XBot::ImuSensor::ConstPtr _imu;

    struct FtHandler
    {
        XBot::ForceTorqueSensor::ConstPtr ft;
        VertexForceOptimizer::UniquePtr vertex_opt;
        std::vector<XBot::Cartesian::CartesianTask::Ptr> vertex_tasks;
    };

    std::vector<FtHandler> _ft_handler;

};

}

#endif // BASE_ESTIMATION_H