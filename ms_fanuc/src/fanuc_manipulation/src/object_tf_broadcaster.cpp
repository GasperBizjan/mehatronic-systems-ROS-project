#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/GetModelProperties.h"
#include "geometry_msgs/TransformStamped.h"

std::string object_name;

void publishTF(gazebo_msgs::GetModelStateResponse msg)
{
    ROS_INFO("publishTF called.");
    // static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
    // tf::Quaternion q;
    // q.setRPY(0, 0, msg->theta);
    // transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_tf_broadcaster");
    if (argc != 2)
    {
        ROS_ERROR("need object_name as argument");
        return -1;
    };
    object_name = argv[1];
    ROS_INFO("Transform publisher for object %s", object_name.c_str());

    ros::NodeHandle n;
    ros::ServiceClient prop = n.serviceClient<gazebo_msgs::GetModelProperties>("/gazebo/get_model_properties");
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    static tf::TransformBroadcaster br;
    ros::Rate rate(10);

    tf::Transform tr;

    gazebo_msgs::GetModelProperties pr;
    pr.request.model_name = object_name;

    prop.waitForExistence();
    ROS_INFO("Service exists");
    std::string object_body_name;
    ros::Duration(3).sleep();
    if (prop.call(pr))
    {
        object_body_name = pr.response.body_names[0];
        ROS_INFO("Received: %s", pr.response.body_names[0].c_str());
    }
    else
    {
        ROS_INFO("Failed to call service /gazebo/get_model_properties");
    }

    gazebo_msgs::GetModelState srv;
    srv.request.model_name = object_name;
    srv.request.relative_entity_name = "world";

    while (ros::ok)
    {
        if (client.call(srv))
        {
            tr.setOrigin(tf::Vector3(srv.response.pose.position.x, srv.response.pose.position.y, srv.response.pose.position.z));
            tf::Quaternion q;
            q.setX(srv.response.pose.orientation.x);
            q.setY(srv.response.pose.orientation.y);
            q.setZ(srv.response.pose.orientation.z);
            q.setW(srv.response.pose.orientation.w);
            tr.setRotation(q);
            br.sendTransform(tf::StampedTransform(tr, ros::Time::now(), "world", object_body_name));
        }
        else
        {
            ROS_ERROR("Failed to call service /gazebo/get_model_state");
            return 1;
        }
    }

    return 0;
};