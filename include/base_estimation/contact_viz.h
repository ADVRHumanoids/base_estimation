#ifndef _CONTACT_VIZ_H_
#define _CONTACT_VIZ_H_

#include <visualization_msgs/MarkerArray.h>
#include <xbot2/ros/ros_support.h>
#include <Eigen/Dense>


namespace ikbe
{
class contact_viz
{
public:
    contact_viz(const std::string& topic_name, XBot::RosSupport* ros);

    bool publish(const std::vector<std::string>& frames, const Eigen::VectorXd& normal_forces);

    bool publish(const std::map<std::vector<std::string>, Eigen::VectorXd>& map);

private:
    visualization_msgs::MarkerArray _marker_array_msg;

    XBot::PublisherPtr<visualization_msgs::MarkerArray> _pub;

    XBot::Journal _j;
};
}

#endif
