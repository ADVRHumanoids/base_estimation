#include "base_estimation/contact_estimation.h"
#include <cmath>
#include <stdexcept>

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
    if(!_contact_state && f_n > _attach_thr)
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
    : _nodehandle(nodeHandle), _current_mode(15), _contact_state(true), _contact_index(vertices2ContactIndex(vertices_name))
    , _previous_contact_state(true)
{
    _mpc_observation_sub = _nodehandle.subscribe("/legged_robot_mpc_observation", 1,
                                                 &ContactPreplanned::mpcObservationCallback, this,
                                                 ros::TransportHints().tcpNoDelay());
}

/* *************************************************************************
 * *************************************************************************
 * *************************************************************************/
int ContactPreplanned::vertices2ContactIndex(std::vector<std::string> vertices_name) {
    int contact_index;
    if (vertices_name.at(0) == "contact_1")
        contact_index = 0;
    else if (vertices_name.at(0) == "contact_2")
        contact_index = 1;
    else if (vertices_name.at(0) == "contact_3")
        contact_index = 2;
    else if (vertices_name.at(0) == "contact_4")
        contact_index = 3;
    else
        std::cout << "Unknown vertice name" << std::endl;
    return contact_index;
}

/* *************************************************************************
 * *************************************************************************
 * *************************************************************************/
void ContactPreplanned::mpcObservationCallback(const ocs2_msgs::mpc_observationConstPtr& msg) {
    _current_mode = msg->mode;
//    update();
    // check that within the callback even should not be returned. callback should just update the flags
    contact_flag_t contacts_state = modeNumber2StanceLeg(_current_mode);
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
contact_flag_t ContactPreplanned::modeNumber2StanceLeg(int modeNumber) {
    contact_flag_t stanceLegs;  // {LF, RF, LH, RH}

    switch (modeNumber) {
      case 0:
        stanceLegs = contact_flag_t{false, false, false, false};
        break;  // 0:  0-leg-stance
      case 1:
        stanceLegs = contact_flag_t{false, false, false, true};
        break;  // 1:  RH
      case 2:
        stanceLegs = contact_flag_t{false, false, true, false};
        break;  // 2:  LH
      case 3:
        stanceLegs = contact_flag_t{false, false, true, true};
        break;  // 3:  RH, LH
      case 4:
        stanceLegs = contact_flag_t{false, true, false, false};
        break;  // 4:  RF
      case 5:
        stanceLegs = contact_flag_t{false, true, false, true};
        break;  // 5:  RF, RH
      case 6:
        stanceLegs = contact_flag_t{false, true, true, false};
        break;  // 6:  RF, LH
      case 7:
        stanceLegs = contact_flag_t{false, true, true, true};
        break;  // 7:  RF, LH, RH
      case 8:
        stanceLegs = contact_flag_t{true, false, false, false};
        break;  // 8:  LF,
      case 9:
        stanceLegs = contact_flag_t{true, false, false, true};
        break;  // 9:  LF, RH
      case 10:
        stanceLegs = contact_flag_t{true, false, true, false};
        break;  // 10: LF, LH
      case 11:
        stanceLegs = contact_flag_t{true, false, true, true};
        break;  // 11: LF, LH, RH
      case 12:
        stanceLegs = contact_flag_t{true, true, false, false};
        break;  // 12: LF, RF
      case 13:
        stanceLegs = contact_flag_t{true, true, false, true};
        break;  // 13: LF, RF, RH
      case 14:
        stanceLegs = contact_flag_t{true, true, true, false};
        break;  // 14: LF, RF, LH
      case 15:
        stanceLegs = contact_flag_t{true, true, true, true};
        break;  // 15: 4-leg-stance
    }

    return stanceLegs;
}

/* *************************************************************************
 * *************************************************************************
 * *************************************************************************/
bool ContactPreplanned::getContactState() const {
    return _contact_state;
}



