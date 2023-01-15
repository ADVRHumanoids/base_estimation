#include "base_estimation/contact_estimation.h"
#include <cmath>
#include <stdexcept>

using namespace ikbe;

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

bool ContactEstimation::getContactState() const
{
    return _contact_state;
}


