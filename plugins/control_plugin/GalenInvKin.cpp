//
// Created by ciis on 1/21/22.
//

#include <iostream>
#include "GalenInvKin.h"

using namespace std;

void GalenInvKin::_compute_inv_kin(chai3d::cTransform& tool_cp, vector<double>& joints) {
    /*
     * forward kinematics matrix E = [R1 * R2 * R3, R1 * R2 * P3 + R1 * P2 + P1], where R3 is identity matrix
     * t_d * E = t_t
     * hence, t_d = (t_t - P_E) * inv(R_E)
     */

    // first computer mobile platform cartesian position using inverse kinematics
    Eigen::Matrix3d rot_roll;
    rot_roll = Eigen::AngleAxisd(joints[3], Eigen::Vector3d::UnitX());
    Eigen::Matrix3d rot_tool;
    rot_tool = Eigen::AngleAxisd(joints[4], Eigen::Vector3d::UnitY());

    /*
    std::cerr << joints[3] << std::endl;
    std::cerr << joints[4] << std::endl;
    std::cerr << rot_roll << std::endl;
    std::cerr << rot_tool << std::endl;*/

    Eigen::Vector3d p_tip  = Eigen::Vector3d(galenSimpModel.lt_mm, 0, -galenSimpModel.ht_mm);
    Eigen::Vector3d p_tool = Eigen::Vector3d(galenSimpModel.la_mm, 0, galenSimpModel.ha_mm);
    Eigen::Vector3d p_roll = Eigen::Vector3d(0, 0, galenSimpModel.hr_mm);
    Eigen::Vector3d translation = rot_roll * rot_tool * p_roll + rot_roll * p_tool + p_tip;

    Eigen::Vector3d p_measured = Eigen::Vector3d(tool_cp.getLocalPos().get(0), tool_cp.getLocalPos().get(1), tool_cp.getLocalPos().get(2));
    Eigen::Vector3d mobile_cp  = (rot_roll * rot_tool).inverse() * (p_measured - translation);

    // compute base joint angle
    for (unsigned int i = 0; i < 3; i++) {
        double x_i = (galenSimpModel.rb_mm - galenSimpModel.rd_mm) * cos((2 * i - 1) * chai3d::C_PI);
        double y_i = (galenSimpModel.rb_mm - galenSimpModel.rd_mm) * sin((2 * i - 1) * chai3d::C_PI);
        joints[i] = mobile_cp.z() - sqrt(pow(galenSimpModel.pl_mm, 2) - pow(mobile_cp.x() - x_i, 2) - pow(mobile_cp.y() - y_i, 2));
    }
}