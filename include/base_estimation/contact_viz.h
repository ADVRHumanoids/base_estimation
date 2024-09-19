#ifndef _CONTACT_VIZ_H_
#define _CONTACT_VIZ_H_

#include <visualization_msgs/msg/marker_array.hpp>
#include <xbot2/ros/ros2_support.h>
#include <Eigen/Dense>


namespace ikbe
{

class contact_viz
{

public:

    contact_viz(const std::string& topic_name, XBot::Ros2Support* ros);

    bool publish(const std::vector<std::string>& frames,
                 const Eigen::VectorXd& normal_forces);

    bool publish(const std::map<std::vector<std::string>, std::vector<double>>& map);

private:

    visualization_msgs::msg::MarkerArray _marker_array_msg;

    XBot::PublisherPtr<visualization_msgs::msg::MarkerArray> _pub;

    rclcpp::Node::SharedPtr _node;

    XBot::Journal _j;
};

}

#endif
