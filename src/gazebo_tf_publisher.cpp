#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf2_msgs/TFMessage.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_tf_publisher");

    ros::NodeHandle nh, nhpr("~");

    std::string gz_link;
    if(!nhpr.getParam("gz_link", gz_link))
    {
        ROS_ERROR("~gz_link parameter required");
        exit(1);
    }

    std::string tf_link;
    if(!nhpr.getParam("tf_link", tf_link))
    {
        ROS_ERROR("~tf_link parameter required");
        exit(1);
    }

    std::string world;
    if(!nhpr.getParam("world", world))
    {
        ROS_ERROR("~world parameter required");
        exit(1);
    }

    ros::Publisher tf_msg_pub = nh.advertise<tf2_msgs::TFMessage>("/tf", 1);

    ros::Time last_pub_time;

    ros::Subscriber gz_link_states_sub = nh.subscribe<gazebo_msgs::LinkStates>(
                "/gazebo/link_states", 1,
                [&](const gazebo_msgs::LinkStatesConstPtr& msg)
    {
        auto now = ros::Time::now();

        if(now == last_pub_time)
        {
            return;
        }

        last_pub_time = now;

        int idx = std::find(msg->name.begin(), msg->name.end(), gz_link) - msg->name.begin();

        if(idx == msg->name.size())
        {
            ROS_ERROR("gazebo link '%s' not found", gz_link.c_str());
            ros::Duration(1.0).sleep();
            return;
        }

        tf2_msgs::TFMessage tf_msg;
        geometry_msgs::TransformStamped tf;

        tf.header.stamp = now;
        tf.header.frame_id = tf_link;
        tf.child_frame_id = world;

        Eigen::Affine3d T;
        tf::poseMsgToEigen(msg->pose[idx], T);
        T = T.inverse();
        tf::transformEigenToMsg(T, tf.transform);

        tf_msg.transforms.push_back(tf);
        tf_msg_pub.publish(tf_msg);


    });

    ros::spin();
}
