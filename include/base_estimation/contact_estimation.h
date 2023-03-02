#ifndef __IKBE_CONTACT_ESTIMATION_H__
#define __IKBE_CONTACT_ESTIMATION_H__

#include <memory>
#include <ros/ros.h>

#include <ocs2_msgs/mpc_observation.h>

namespace ikbe
{

/**
 * @brief The ContactEstimation class implements a minimalistic
 * Schmitt Trigger to estimate the contact state based on normal
 * force measurements
 */
class ContactEstimation
{

public:

    typedef std::unique_ptr<ContactEstimation> UniquePtr;

    enum class Event
    {
        Released,
        Attached,
        None
    };

    ContactEstimation(double release_thr,
                      double attach_thr);

    Event update(double f_n);

    bool getContactState() const;

private:

    double _release_thr;
    double _attach_thr;
    bool _contact_state;


};


/**
 * @brief The ContactPreplanned class sets the active contacts based on inputs from
 * a motion planner. It is not based on force estimation.
 */
class ContactPreplanned
{

    // TODO:This class is defined for one point contact. However is subscribes to information for all the contacts.
    // Thus this class can be changed to represent all the contacts of the robot but corresponding changes have to be
    // done on BaseEstimation as well as BaseEstimationNode classes.
public:
    typedef std::unique_ptr<ContactPreplanned> UniquePtr;

    enum class Event
    {
        Released,
        Attached,
        None
    };

    /*!
     * \brief ContactPreplanned constructor
     * \param nodeHandle ros nodehandle to subscribe to the contact observed in the MPC formulation
     * \param vertices_name the vertices of the contact, for centauro it is only the frame of the point contact
     */
    ContactPreplanned(ros::NodeHandle& nodeHandle, std::vector<std::string> vertices_name);

    // update the contact state of the _contact_index
    Event update();

    // returns the contact state of the _contact_index
    bool getContactState() const;

private:
    void mpcObservationCallback(const ocs2_msgs::mpc_observationConstPtr& msg);

    // maps vertice name (for centauro the point contact frame) to contact index from 0 to 3
    int vertices2ContactIndex(std::vector<std::string> vertices_name);

    ros::Subscriber _mpc_observation_sub;
    ros::NodeHandle _nodehandle;

    int _current_mode;               // mode is defined in ocs2 as an integer that describes the contact status of all leg contacts
    bool _contact_state;             // contact status of the _contact_index
    bool _previous_contact_state;    // previous contact status of the _contact_index
    int _contact_index;              // the one contact for which this class is defined, from 0 to 3
};
}   // namespace ikbe
#endif // CONTACT_ESTIMATION_H
