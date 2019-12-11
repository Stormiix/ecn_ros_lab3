#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <ecn_common/token_handle.h>
#include <ros/ros.h>
#include <vector>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <baxter_core_msgs/SolvePositionIK.h>

//Global variables
baxter_core_msgs::JointCommand cmd_msg; // Command message
ros::Publisher pub_command ;//The publisher of the command to move the joint
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer);
baxter_core_msgs::SolvePositionIK srv;
geometry_msgs::TransformStamped transformStamped;
geometry_msgs::PoseStamped poseOut;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "ninja_node");
    ros::NodeHandle node("~"); // Use local namespace
    ecn::TokenHandle token("Groupe4");
    ros::Rate rate(10.0);

    // Publisher declaration
    pub_command = node.advertise<baxter_core_msgs::JointCommand>("/joint_command", 1);

    // IK solver
    ros::ServiceClient client = node.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/left/PositionKinematicsNode/IKService");

    // Prepare cmd_msg to control in position the left arm
    cmd_msg.mode = cmd_msg.POSITION_MODE; 

    while (node.ok()){
        try{
          transformStamped = tfBuffer.lookupTransform("base", "ninja_gripper", ros::Time(0));
          ROS_INFO("Found the frame transform.");
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // Convert the transformStamped to a PoseStamped
        poseOut.header = transformStamped.header;
        poseOut.pose.position.x = transformStamped.transform.translation.x;
        poseOut.pose.position.y = transformStamped.transform.translation.y;
        poseOut.pose.position.z = transformStamped.transform.translation.z;
        poseOut.pose.orientation = transformStamped.transform.rotation;
        
        // Set the request and call the IKsolver
        srv.request.pose_stamp.clear();
        srv.request.pose_stamp.push_back(poseOut);

        if(client.call(srv)){
            ROS_INFO("Called the service successfully.");
            if(srv.response.result_type[0] == 0){
                ROS_WARN("No solution for the set pose!");
            }else{
                // Set the command joint positions & names
                cmd_msg.names = srv.response.joints[0].name;
                cmd_msg.command = srv.response.joints[0].position; 
                pub_command.publish(cmd_msg); // Publish the command
                ROS_INFO("Published the command successfully.");
            }
        }

        // update token, sync with sampling time and activate publish / subscribe
        token.update();
        rate.sleep();
        ros::spinOnce();
    }
}

