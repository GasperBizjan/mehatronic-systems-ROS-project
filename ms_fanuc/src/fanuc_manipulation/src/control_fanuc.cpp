#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_srvs/SetBool.h>
#include <gazebo_ros_link_attacher/Attach.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial"); //inicijacija vozlišča
    ros::NodeHandle node_handle;
    // Klienti, ki kličejo servise:
    // - "/gripper/gripper_closed" (zapiranje/odpiranje)
    // - "/link_attacher_node/attach" (priklop palice na roko)
    // - "/link_attacher_node/detach" (odklop palice od roke)
    ros::ServiceClient gripper_client = node_handle.serviceClient<std_srvs::SetBool>("/gripper/gripper_closed");
    ros::ServiceClient attach_object_client = node_handle.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    ros::ServiceClient detach_object_client = node_handle.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
    std_srvs::SetBool gripper_info;
    gazebo_ros_link_attacher::Attach attach_object_info;
    gazebo_ros_link_attacher::Attach detach_object_info;
    // Spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Inicjalizacija in definicija načrtovalne skupine za "move_base"
    static const std::string PLANNING_GROUP = "fanuc_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Inicjalizacija "Rviz Visual Tools"
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();// Počisti obstoječe markerje
    visual_tools.loadRemoteControl();// Naloži ročno krmiljenje

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1;
    visual_tools.publishText(text_pose, "MoveGroupInterface Tutorial", rviz_visual_tools::LIME_GREEN, rviz_visual_tools::XLARGE);
    visual_tools.trigger();
    /////////////////////
    // JEDRO PROGRAMA  //
    /////////////////////

    // 0.) Pomnenje začetne pozicije
    // "RobotState" je objekt z informacijami o trenutni postavitvi robota.
    moveit::core::RobotStatePtr starting_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    starting_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // 1.) Premik do palice:
    geometry_msgs::Pose target_pose;
    // - definiramo prvi pomik
    target_pose.position.x = 0.2;
    target_pose.position.y = 0.3;
    target_pose.position.z = 0.3;
    target_pose.orientation.y = sqrt(2) / 2;
    target_pose.orientation.w = sqrt(2) / 2;
    move_group.setPoseTarget(target_pose);
    ROS_INFO_NAMED("move_to_pose", "Setting the target position to x=%g, y=%g, z=%g", target_pose.position.x, target_pose.position.y, target_pose.position.z);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
    // - izvedba pomika
    move_group.move();
    // - definiramo drugi pomik
    target_pose.position.z = 0.202;
    move_group.setPoseTarget(target_pose);
    ROS_INFO_NAMED("move_to_pose", "Setting the target position to x=%g, y=%g, z=%g", target_pose.position.x, target_pose.position.y, target_pose.position.z);
    ros::Duration(0.1).sleep();
    // - izvedba drugega pomika
    move_group.move();

    // 2.) Zapiranje griperja + zaklep prostoste stopnje palice:
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
    gripper_info.request.data = true; //Nastavimo, da je gripper zaprt
    gripper_client.call(gripper_info); //klient kliče servis in preda informacijo o gripperju
    ROS_INFO_STREAM("Box gripped");
    ros::Duration(0.1).sleep();
    attach_object_info.request.model_name_1 = "robot"; //Nastavimo povezavo med griperjem in palico
    attach_object_info.request.link_name_1 = "right_finger";
    attach_object_info.request.model_name_2 = "box";
    attach_object_info.request.link_name_2 = "my_box";
    attach_object_client.call(attach_object_info); //klient kliče servis in preda informacijo o povezavi
    ROS_INFO_STREAM("Box attached.");

    // 3.) Palico premaknemo na novo mesto
    // - definiramo prvi pomik
    target_pose.position.z = 0.3;
    move_group.setPoseTarget(target_pose);
    ROS_INFO_NAMED("move_to_pose", "Setting the target position to x=%g, y=%g, z=%g", target_pose.position.x, target_pose.position.y, target_pose.position.z);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
    // - izvedba pomika
    move_group.move();
    // - definiramo drugi pomik
    target_pose.position.x = -0.2;
    target_pose.position.y = -0.3;
    move_group.setPoseTarget(target_pose);
    ROS_INFO_NAMED("move_to_pose", "Setting the target position to x=%g, y=%g, z=%g", target_pose.position.x, target_pose.position.y, target_pose.position.z);
    ros::Duration(0.1).sleep();
    // - izvedba drugega pomika
    move_group.move();
    // - definiramo tretji pomik
    target_pose.position.z = 0.202;
    move_group.setPoseTarget(target_pose);
    ROS_INFO_NAMED("move_to_pose", "Setting the target position to x=%g, y=%g, z=%g", target_pose.position.x, target_pose.position.y, target_pose.position.z);
    ros::Duration(0.1).sleep();
    // - izvedba tretjega pomika
    move_group.move();

    // 4.) Odklep prostoste stopnje palice + odpiranje griperja:
    detach_object_info.request.model_name_1 = "robot"; //Odstranimo povezavo med griperjem in palico
    detach_object_info.request.link_name_1 = "right_finger";
    detach_object_info.request.model_name_2 = "box";
    detach_object_info.request.link_name_2 = "my_box";
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
    detach_object_client.call(detach_object_info); //klient kliče servis in preda informacijo o povezavi
    ROS_INFO_STREAM("Box detached");
    gripper_info.request.data = false; //Nastavimo, da je gripper odprt
    ros::Duration(0.1).sleep();
    gripper_client.call(gripper_info); //klient kliče servis in preda informacijo o gripperju
    ROS_INFO_STREAM("Box gripped");

    // 5.) Zadnji pomik
    // - definiramo pomik
    target_pose.position.z = 0.3;
    move_group.setPoseTarget(target_pose);
    ROS_INFO_NAMED("move_to_pose", "Setting the target position to x=%g, y=%g, z=%g", target_pose.position.x, target_pose.position.y, target_pose.position.z);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
    // - izvedba pomika
    move_group.move();

    //!.) Premik na začetek:
    ros::Duration(0.1).sleep();
    move_group.setJointValueTarget(joint_group_positions);
    move_group.move();

    /////////////////////
    // KONEC PROGRAMA  //
    /////////////////////

    ros::shutdown();
    return 0;
}