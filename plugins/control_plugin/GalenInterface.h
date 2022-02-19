//
// Created by Jintan Zhang on 1/17/2022.
//

#ifndef GALEN_SIMULATION_AMBF_GALENINTERFACE_H
#define GALEN_SIMULATION_AMBF_GALENINTERFACE_H

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <math/CTransform.h>

#include <plugin_msgs/RobotState.h>


using namespace chai3d;
using namespace std;

class GalenInterface {

private:
    // robot states
    vector<double> measured_jp;    // measured joint position
    vector<double> measured_jv;    // measured joint velocity

    cTransform     mobile_cp;      // mobile platform cartesian position
    cTransform     tool_cp;        // tool cartesian position

    // TODO: implement
    vector<double> measured_tra_jv;    // measured translational joint velocities
    vector<double> measured_rot_jv;    // measured rotational joint velocities
    vector<double> measured_cf;        // measured cartesian force

    // robot states subscribers
    ros::Subscriber sub_measured_jp;
    ros::Subscriber sub_mobile_cp;
    ros::Subscriber sub_tool_cp;

    ros::Subscriber sub_measured_tra_jv;
    ros::Subscriber sub_measured_rot_jv;
    ros::Subscriber sub_measured_cf;

    // robot states publisher, to align actual robot
    ros::Publisher pub_servo_jp;

public:
    GalenInterface();
    ~GalenInterface();

    // initialization function: init ros sub and pub
    ros::NodeHandle* ros_node;

    // subscriber callback functions
    void _mobile_cp_CB(geometry_msgs::TransformStampedConstPtr);
    void _tool_cp_CB(geometry_msgs::TransformStampedConstPtr);
    // void _measured_jp_CB(sensor_msgs::JointStateConstPtr);
    void _measured_jp_CB(plugin_msgs::RobotStateConstPtr);

    void _measured_tra_jv_CB(geometry_msgs::Vector3ConstPtr);
    void _measured_rot_jv_CB(geometry_msgs::Vector3ConstPtr);
    void _measured_cf_CB(geometry_msgs::WrenchStampedConstPtr);

    // publisher functions
    void _servo_jp(vector<double> joints);

    // access functions
    vector<double>& get_measured_jp();
    vector<double>& get_measured_jv();
    vector<double>& get_measured_tra_jv();
    vector<double>& get_measured_rot_jv();
    vector<double>& get_measured_cf();
    cTransform&     get_mobile_cp();
    cTransform&     get_tool_cp();

    // publisher message
    sensor_msgs::JointState msg_servo_jp;
};


#endif //GALEN_SIMULATION_AMBF_GALENINTERFACE_H
