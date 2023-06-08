#ifndef __IK_BASE_ESTIMATION_H__
#define __IK_BASE_ESTIMATION_H__

#include "vertex_force_optimizer.h"
#include "contact_estimation.h"

#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/problem/Postural.h>
#include <cartesian_interface/utils/estimation/ForceEstimation.h>

#include <XBotInterface/Utils.h>

namespace ikbe
{

/**
 * @brief The BaseEstimation class performs estimation
 * of the floating base state based on force-torque sensing
 * (or estimation) as well as imu readings.
 */
class BaseEstimation
{

public:

    typedef std::unique_ptr<BaseEstimation> UniquePtr;

    /**
     * @brief The Options struct contains parameters
     * for the BaseEstimation class, to be provided
     * upon construction
     */
    struct Options
    {
        /**
         * @brief period at which update() is called
         */
        double dt;

        /**
         * @brief enables/disables the generation of
         * .mat log files upon destruction
         */
        bool log_enabled;

        /**
         * @brief estimate_contacts enables/disables the estimation of contact points
         * In the disabled case the contacts are considered to be known (e.g. from the motion planner)
         */
        bool estimate_contacts;

        /**
         * @brief contact release threshold for contact
         * estimation
         */
        double contact_release_thr;

        /**
         * @brief contact attachment threshold for contact
         * estimation
         */
        double contact_attach_thr;

        Options();
    };

    /**
     * @brief BaseEstimation class constructor with contacts estimation
     * @param model is a ModelInterface that is externally kept updated with the robot state
     * @param est_model_pb is a yaml cartesio stack of tasks describing the contact and sensor model for the robot
     * @param opt is a struct of options
     */
    BaseEstimation(XBot::ModelInterface::Ptr model, YAML::Node est_model_pb, Options opt = Options());

    /**
     * @brief BaseEstimation class constructor with contacts to be subscribed from ros (instead of estimated)
     * @param model is a ModelInterface that is externally kept updated with the robot state
     * @param est_model_pb is a yaml cartesio stack of tasks describing the contact and sensor model for the robot
     * @param nodehandle is necessary to be able to subscribe to the contacts planned by the planner through ContactPreplanned
     * @param opt is a struct of options
     */
    BaseEstimation(XBot::ModelInterface::Ptr model, YAML::Node est_model_pb,
                   ros::NodeHandle& nodehandle, Options opt = Options());

    /**
     * @brief returns options associated with the estimator
     */
    Options getOptions() const;

    /**
     * @brief returns the vector of reference frame names for estimated wrenches
     */
    std::vector<std::string> getEstimatedWrenchReferenceFrames() const { return _estimatedWrenchRefFrames;}

    /**
     * @brief ci returns the internal cartesio object
     */
    XBot::Cartesian::CartesianInterfaceImpl::Ptr ci() const;

    /**
     * @brief add an imu sensor belonging to the robot to
     * the estimation process
     */
    void addImu(XBot::ImuSensor::ConstPtr imu);

    /**
     * @brief usesImu returns true if the estimator is using
     * imu information
     */
    bool usesImu() const;

    /**
     * @brief imu returns the imu shared pointer associated
     * with the base estimator, nullptr if addImu() was not
     * called
     */
    XBot::ImuSensor::ConstPtr imu() const;

    /**
     * @brief add a virtual ft sensor on the given robot link,
     * based on an internal force estimator
     * @param link_name is the robot link where the virtual ft
     * is placed
     * @param dofs is a vector of indices specifying which wrench
     * components should be estimated (e.g. {0, 1, 2} for pure force)
     * @param contact_points is a list of vertices whose
     * convex hull is a representation of the contact rface;
     * e.g. (i) the four corner frames of a square foot, or
     * (ii) the single point contact frame for a point contact
     * @return a shared pointer to the created ft, to be used as input
     * to addXXXContact functions
     */
    XBot::ForceTorqueSensor::ConstPtr createVirtualFt(std::string link_name,
                                                      std::vector<int> dofs);

    /**
     * @brief adds a surface contact to the estimator contact model
     * @param vertex_frames is a list of urdf frames describing
     * the contact surface convex hull
     * @param ft is a force torque sensor pointer to be used for
     * contact estimation
     */
    void addSurfaceContact(std::vector<std::string> vertex_frames,
                           XBot::ForceTorqueSensor::ConstPtr ft);

    /**
     * @brief adds a rolling contact to the estimator contact model
     * @param wheel_name is the urdf name of the wheel link
     * @param ft is a force torque sensor pointer to be used for
     * contact estimation
     */
    void addRollingContact(std::string wheel_name,
                           XBot::ForceTorqueSensor::ConstPtr ft);

    /**
     * @brief update
     * @param pose estimated base pose
     * @param vel estimated filtered base velocity
     * @param raw_vel estimated base velocity
     * @return
     */
    bool update(Eigen::Affine3d& pose,
                Eigen::Vector6d& vel,
                Eigen::Vector6d& raw_vel);

    /**
     * @brief reset the whole ci
     */
    void reset();

    /**
     * @brief reset a single task
     * @param task_name name of the task
     */
    bool reset(const std::string& task_name);

    /* Velocity filter parameters */
    void setFilterOmega(const double omega);
    void setFilterDamping(const double eps);
    void setFilterTs(const double ts);

    /* Contact information */
    struct ContactInformation
    {
        std::string name;
        Eigen::Vector6d wrench;
        std::vector<std::string> vertex_frames;
        std::vector<double> vertex_weights;
        bool contact_state;
        bool contact_haptic_state;   // from haptic, equal to contact_state if estimate_contacts = true

        ContactInformation(std::string name, std::vector<std::string> vertex_frames);
    };

    std::vector<ContactInformation> contact_info;

private:
    ros::NodeHandle _nodehandle;    // To subscribe to contacts of the planner
    std::vector<std::string> _estimatedWrenchRefFrames;
    Options _opt;

    Eigen::VectorXd _q, _qdot;
    XBot::ModelInterface::Ptr _model;
    XBot::Cartesian::CartesianInterfaceImpl::Ptr _ci;
    XBot::Cartesian::PosturalTask::Ptr _postural;

    XBot::Cartesian::CartesianTask::Ptr _imu_task;
    XBot::ImuSensor::ConstPtr _imu;

    XBot::Cartesian::Utils::ForceEstimation::Ptr _fest;

    struct ContactHandler
    {
        std::vector<std::string> vertex_frames;
        XBot::ForceTorqueSensor::ConstPtr ft;
        VertexForceOptimizer::UniquePtr vertex_opt;
        std::vector<XBot::Cartesian::TaskDescription::Ptr> vertex_tasks;
        ContactEstimation::UniquePtr contact_est;
        ContactPreplanned::UniquePtr contact_planned;       // ContactPreplanned addition

    };

    std::vector<ContactHandler> _contact_handler;

    double _alpha;
    Eigen::VectorXd _weights;

    XBot::Utils::SecondOrderFilter<Eigen::Vector6d>::Ptr _vel_filter;

    XBot::MatLogger2::Ptr _logger;

    void handle_contact_switch(ContactHandler& fth);
    void handle_preplanned_contact_switch(ContactHandler& fth); // handle contact switch but for preplanned contacts


};

}

#endif // BASE_ESTIMATION_H
