#include "base_estimation/contact_estimation.h"
#include <cmath>
#include <stdexcept>

#include "common.h"


using namespace ikbe;

/* *************************************************************************
 * *************************************************************************
 * *************************************************************************/
ContactEstimation::ContactEstimation(double release_thr,
                                     double attach_thr):
    _release_thr(release_thr),
    _attach_thr(attach_thr),
    _contact_state(true)
{
    if(_release_thr > _attach_thr)
    {
        throw std::invalid_argument("ContactEstimation: release_thr > attach_thr");
    }
}

/* *************************************************************************
 * *************************************************************************
 * *************************************************************************/
ContactEstimation::Event ContactEstimation::update(double f_n)
{
    // take abs val
    f_n = std::fabs(f_n);

    // contact to be deactivated
    if(_contact_state && f_n < _release_thr)
    {
        _contact_state = false;
        return Event::Released;
    }

    // contact to be activated
    else if(!_contact_state && f_n > _attach_thr)
    {
        _contact_state = true;
        return Event::Attached;
    }

    // nothing happened
    return Event::None;
}

/* *************************************************************************
 * *************************************************************************
 * *************************************************************************/
bool ContactEstimation::getContactState() const
{
    return _contact_state;
}

/* *************************************************************************
 * ************************ ContactPreplanned ******************************
 * *************************************************************************/
ContactPreplanned::ContactPreplanned(ros::NodeHandle& nodeHandle, std::vector<std::string> vertices_name)
    : _nodehandle(nodeHandle), _current_mode(15), _contact_state(true), _previous_contact_state(true)
{
    _contact_index = vertices2ContactIndex(vertices_name);
    _mpc_observation_sub = _nodehandle.subscribe("/legged_robot_mpc_observation", 1,
                                                 &ContactPreplanned::mpcObservationCallback, this,
                                                 ros::TransportHints().tcpNoDelay());
    if (ikbe_common::isArm(vertices_name.front(), _nodehandle)) {
        _contact_state = false;
        _previous_contact_state = false;
    }
}

/* *************************************************************************
 * *************************************************************************
 * *************************************************************************/
int ContactPreplanned::vertices2ContactIndex(std::vector<std::string> vertices_name) {

    std::map<std::string, std::string> surface_contacts, arm_surface_contacts;
    _nodehandle.getParam("surface_contacts", surface_contacts);                              // first get feet
    _nodehandle.getParam("arm_surface_contacts", arm_surface_contacts);                         // then arms

    int contact_index;
    if (ikbe_common::isArm(vertices_name.front(), _nodehandle)) {     // for arm contact
        contact_index = 4;
        for (auto& frame : arm_surface_contacts) {
            if (vertices_name.front() == frame.first)
                break;
            contact_index++;
        }
    } else {                                // for feet contact
        contact_index = 0;
        for (auto& frame : surface_contacts) {
            if (vertices_name.front() == frame.first)
                break;
            contact_index++;
        }
    }

    return contact_index;
}

/* *************************************************************************
 * *************************************************************************
 * *************************************************************************/
void ContactPreplanned::mpcObservationCallback(const ocs2_msgs::mpc_observationConstPtr& msg) {
    _current_mode = msg->mode;
//    update();
    // check that within the callback even should not be returned. callback should just update the flags
    auto contacts_state = ocs2::legged_robot::modeNumber2ActiveContacts(_current_mode);
    _contact_state = contacts_state.at(_contact_index);
}


/* *************************************************************************
 * *************************************************************************
 * *************************************************************************/
ContactPreplanned::Event ContactPreplanned::update()
{
//    contact_flag_t contacts_state = modeNumber2StanceLeg(_current_mode);
//    _contact_state = contacts_state.at(_contact_index);

//    std::cout << "Contact n. " << _contact_index << " Previous cs Current state: " << _previous_contact_state << " -- " << _contact_state <<  std::endl;
    // contact to be deactivated
    if(_contact_state != _previous_contact_state)
    {
        _previous_contact_state = _contact_state;

        if (_contact_state) {
//            std::cout << "Contact attached event! " << std::endl;
            return Event::Attached;     // new made contact
        }
        else {
//            std::cout << "Contact released event! " << std::endl;
            return Event::Released;     // new broken contact
        }
    }

    else
    {
        _previous_contact_state = _contact_state;
        return Event::None; // nothing happened
    }
}

/* *************************************************************************
 * *************************************************************************
 * *************************************************************************/
bool ContactPreplanned::getContactState() const {
    return _contact_state;
}



