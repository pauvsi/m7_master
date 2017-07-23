#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
ros::init(argc, argv, "myRobot_move_joint");
ros::NodeHandle n;
ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/arm/jointStates", 1);
tf::TransformBroadcaster broadcaster;
ros::Rate loop_rate(30);

const double degree = M_PI/180;
double rot4 = 1;

geometry_msgs::TransformStamped odom_trans;
sensor_msgs::JointState joint_state;
odom_trans.header.frame_id = "odom";
odom_trans.child_frame_id = "base_link";

joint_state.name.resize(4);
joint_state.position.resize(4);
joint_state.name[0] ="servo1_to_bone1";
joint_state.name[1] ="servo2_to_servo3";
joint_state.name[2] ="servo3_to_bone2";
joint_state.name[3] ="servo4_to_bone3";

while (ros::ok()) {
    //update joint_state
    joint_state.header.stamp = ros::Time::now();
    joint_state.position[0] = 0;
    joint_state.position[1] = 0;
    joint_state.position[2] = 0;
    joint_state.position[3] = 0;//rot4*degree;

    // update transform
    // (moving in a circle with radius=2)
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = 0;
    odom_trans.transform.translation.y = 0;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

    //send the joint state and transform
    joint_pub.publish(joint_state);
    broadcaster.sendTransform(odom_trans);

    rot4 += 1;
    if (rot4 > 90) rot4 = 0;

    loop_rate.sleep();
}
return 0;
}
