#ifndef COMMON_H
#define COMMON_H

#include <cartesian_interface/CartesianInterfaceImpl.h>

namespace ikbe_common
{

/**
 * @brief footFrames get frames associated to a foot
 * @param foot_prefix prefix of a foot
 * @return vector of frames associated to a foot_prefix
 */
inline std::vector<std::string> footFrames(XBot::Cartesian::CartesianInterfaceImpl& ci,
                                           const std::string& foot_prefix)
{
    using namespace XBot::Cartesian;

    std::vector<std::string> feet_tasks;
    for(auto t : ci.getTaskList())
    {
        auto cart = std::dynamic_pointer_cast<CartesianTask>(ci.getTask(t));

        if(!cart)
        {
            continue;
        }

        if(t.length() >= foot_prefix.length() &&
                t.substr(0,foot_prefix.length()) == foot_prefix)
        {
            feet_tasks.push_back(t);
        }
    }

    return feet_tasks;
}

}

#endif // COMMON_H
