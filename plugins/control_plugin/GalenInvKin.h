//
// Created by ciis on 1/21/22.
//

#ifndef GALEN_SIMULATION_AMBF_GALENINVKIN_H
#define GALEN_SIMULATION_AMBF_GALENINVKIN_H

#include <math/CTransform.h>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "GalenSimpModel.h"

using namespace std;

class GalenInvKin {
    /*
     * Inverse Kinematics
     */
public:
    void _compute_inv_kin(chai3d::cTransform& tool_cp, vector<double>& joints);

private:
    GalenSimpModel galenSimpModel;

};


#endif //GALEN_SIMULATION_AMBF_GALENINVKIN_H
