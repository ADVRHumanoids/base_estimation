#include <base_estimation/contact_viz.h>

using namespace ikbe;

contact_viz::contact_viz(const std::string& topic_name, XBot::RosSupport* ros):
    _j("contact_viz")
{
    _pub = ros->advertise<visualization_msgs::MarkerArray>(topic_name, 1);
}

bool contact_viz::publish(const std::map<std::vector<std::string>, std::vector<double>>& map)
{
    _marker_array_msg.markers.clear();

    visualization_msgs::Marker marker;
    ros::Time t = ros::Time::now();

    for(auto element : map)
    {
        std::vector<std::string> frames = element.first;
        std::vector<double> normal_forces = element.second;

        if(frames.size() != normal_forces.size())
        {
            _j.jerror("frames.size() != normal_forces.size() : '{}' != '{}'", frames.size(), normal_forces.size());
            return false;
        }

        for(unsigned int i = 0; i < frames.size(); ++i)
        {
            std::string frame = frames[i];
            double normal_force = normal_forces[i];

            marker.header.frame_id = frame;
            marker.header.stamp = t;
            marker.ns = frame + "_contact";
            marker.id = i;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = 0.0;
            marker.pose.position.y = 0.0;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.;
            marker.pose.orientation.y = -M_PI_2;
            marker.pose.orientation.z = 0.;
            marker.pose.orientation.w = M_PI_2;

            marker.scale.x = normal_force;
            marker.scale.y = 0.005;
            marker.scale.z = 0.005;

            marker.color.a = 1.;
            marker.color.r = 0.;
            marker.color.g = 1.;
            marker.color.b = 0.;

            _marker_array_msg.markers.push_back(marker);
        }
    }

    _pub->publish(_marker_array_msg);

    return true;
}

bool contact_viz::publish(const std::vector<std::string>& frames, const Eigen::VectorXd &normal_forces)
{
    _marker_array_msg.markers.clear();

    if(frames.size() != normal_forces.size())
    {
        _j.jerror("frames.size() != normal_forces.size() : '{}' != '{}'", frames.size(), normal_forces.size());
        return false;
    }

    visualization_msgs::Marker marker;
    ros::Time t = ros::Time::now();
    for(unsigned int i = 0; i < frames.size(); ++i)
    {
        std::string frame = frames[i];
        double normal_force = normal_forces[i];

        marker.header.frame_id = frame;
        marker.header.stamp = t;
        marker.ns = frame + "_contact";
        marker.id = i;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.;
        marker.pose.orientation.y = -M_PI_2;
        marker.pose.orientation.z = 0.;
        marker.pose.orientation.w = M_PI_2;

        marker.scale.x = normal_force;
        marker.scale.y = 0.005;
        marker.scale.z = 0.005;

        marker.color.a = 1.;
        marker.color.r = 0.;
        marker.color.g = 1.;
        marker.color.b = 0.;

        _marker_array_msg.markers.push_back(marker);
    }

    _pub->publish(_marker_array_msg);

    return true;
}
