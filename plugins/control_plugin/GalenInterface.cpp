//
// Created by Jintan Zhang on 1/17/2022.
//

#include "GalenInterface.h"
#include <afFramework.h>

#include <ambf_server/RosComBase.h>

GalenInterface::GalenInterface() {
    // why af ros node?
    ros_node = afROSNode::getNode();

    // initialize subscriber
    // TODO: update topic name
    sub_mobile_cp   = ros_node->subscribe("/Fake_Galen_CP_Publisher", 1, &GalenInterface::_mobile_cp_CB, this);
    sub_tool_cp     = ros_node->subscribe("/Fake_Galen_CP_Publisher", 1, &GalenInterface::_tool_cp_CB, this);

    // Topic name is: /rems/robot_state
    // msg type: REMS/RobotState
    // sub_measured_jp = ros_node->subscribe("/rems/robot_state", 1, &GalenInterface::_measured_jp_CB, this);

    // TMP solutions: use /rems/measured_js from Python node
    sub_measured_jp = ros_node->subscribe("/rems/robot_state", 1, &GalenInterface::_measured_jp_CB, this);

    // TODO: add callback, fix namespace
    // sub_tf_patient2world = ros_node->subscribe("/Fake_Galen_JS_Publisher", 1, &GalenInterface::_measured_jp_CB, this); // get transformation from patient to the OR coordinate frame
    // sub_tf_patient2robot = ros_node->subscribe("/Fake_Galen_JS_Publisher", 1, &GalenInterface::_measured_jp_CB, this); // get transformation from patient to robot (registration result)

    // TODO: add subscriber to joint velocities, need to combine them to a single topic if possible
    //sub_measured_tra_jv = ros_node->subscribe("/rems/joint_velocity_translation", 1, &GalenInterface::_measured_tra_jv_CB, this); // get transformation from patient to robot (registration result)
    //sub_measured_rot_jv = ros_node->subscribe("/rems/joint_velocity_rotation", 1, &GalenInterface::_measured_rot_jv_CB, this); // get transformation from patient to robot (registration result)
    //sub_measured_cf = ros_node->subscribe("/rems/measured_cf", 1, &GalenInterface::_measured_cf_CB, this);

    // initialize publisher
    // TODO: may not be necessary since Galen may not have servo option
    pub_servo_jp = ros_node->advertise<sensor_msgs::JointState>("servo_jp", 1);
}

GalenInterface::~GalenInterface() {
    sub_mobile_cp.shutdown();
    sub_tool_cp.shutdown();
    sub_measured_jp.shutdown();
    sub_measured_cf.shutdown();
    sub_measured_rot_jv.shutdown();
    sub_measured_tra_jv.shutdown();

    pub_servo_jp.shutdown();
}

cTransform& GalenInterface::get_mobile_cp() {
    return mobile_cp;
}

cTransform& GalenInterface::get_tool_cp() {
    return tool_cp;
}

vector<double>& GalenInterface::get_measured_jp() {
    return measured_jp;
}

vector<double>& GalenInterface::get_measured_jv() {
    return measured_jv;
}

vector<double>& GalenInterface::get_measured_tra_jv() {
    return measured_tra_jv;
}

vector<double>& GalenInterface::get_measured_rot_jv() {
    return measured_rot_jv;
}

vector<double>& GalenInterface::get_measured_cf() {
    return measured_cf;
}
// plugin_msgs::RobotStateConstPtr   sensor_msgs::JointStateConstPtr
void GalenInterface::_measured_jp_CB(plugin_msgs::RobotStateConstPtr msg) {
/*
 * ideally, this should receive data from five joints
 */
    measured_jp = msg->joint_position;
    measured_jv = msg->joint_velocity;
    //std::cerr << measured_jp[0] << std::endl;
}

void GalenInterface::_tool_cp_CB(geometry_msgs::TransformStampedConstPtr msg) {
    // set translation part
    tool_cp.setLocalPos(cVector3d(msg->transform.translation.x,
                                        msg->transform.translation.y,
                                        msg->transform.translation.z));

    // set rotation part
    // rotation received is in quaternion, convert to rotation matrix first
    cQuaternion rot(msg->transform.rotation.w,
                    msg->transform.rotation.x,
                    msg->transform.rotation.y,
                    msg->transform.rotation.z);
    cMatrix3d rotM;
    rot.toRotMat(rotM);

    tool_cp.setLocalRot(rotM);
}

void GalenInterface::_mobile_cp_CB(geometry_msgs::TransformStampedConstPtr msg) {
    // set translation part
    mobile_cp.setLocalPos(cVector3d(msg->transform.translation.x,
                                  msg->transform.translation.y,
                                  msg->transform.translation.z));

    // set rotation part
    // rotation received is in quaternion, convert to rotation matrix first
    cQuaternion rot(msg->transform.rotation.w,
                    msg->transform.rotation.x,
                    msg->transform.rotation.y,
                    msg->transform.rotation.z);
    cMatrix3d rotM;
    rot.toRotMat(rotM);

    mobile_cp.setLocalRot(rotM);
}

void GalenInterface::_servo_jp(vector<double> joints) {
    if (joints.size() != 5){
        cerr << "ERROR! IN SERVO JP, JOINT LENGTH MUST BE EQUAL TO 5. IGNORING!" << endl;
        return;
    }

    for (int idx = 0 ; idx < joints.size() ; idx++){
        msg_servo_jp.position[idx] = joints[idx];
    }
    pub_servo_jp.publish(msg_servo_jp);
}

void GalenInterface::_measured_cf_CB(geometry_msgs::WrenchStampedConstPtr msg) {
    measured_cf.push_back(msg->wrench.force.x);
    measured_cf.push_back(msg->wrench.force.y);
    measured_cf.push_back(msg->wrench.force.z);

    measured_cf.push_back(msg->wrench.torque.x);
    measured_cf.push_back(msg->wrench.torque.y);
    measured_cf.push_back(msg->wrench.torque.z);
}

void GalenInterface::_measured_tra_jv_CB(geometry_msgs::Vector3ConstPtr msg) {
    measured_tra_jv.push_back(msg->x);
    measured_tra_jv.push_back(msg->y);
    measured_tra_jv.push_back(msg->z);
}

// TODO: need to figure out if this is vector
void GalenInterface::_measured_rot_jv_CB(geometry_msgs::Vector3ConstPtr msg) {
    measured_rot_jv.push_back(msg->x);
    measured_rot_jv.push_back(msg->y);
}

