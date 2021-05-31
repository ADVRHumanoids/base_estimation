#ifndef __IKBE_CONTACT_ESTIMATION_H__
#define __IKBE_CONTACT_ESTIMATION_H__

#include <memory>

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

}


#endif // CONTACT_ESTIMATION_H
