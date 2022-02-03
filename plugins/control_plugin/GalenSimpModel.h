//
// Created by ciis on 1/22/22.
//

#ifndef GALEN_SIMULATION_AMBF_GALENSIMPMODEL_H
#define GALEN_SIMULATION_AMBF_GALENSIMPMODEL_H

#include <vector>

using namespace std;

class GalenSimpModel {

public:
    double rb_mm = 1;
    double rd_mm = 1;
    double pl_mm = 1;          // the length of the delta parallelograms, i.e. pl
    double hr_mm = 1;          // delta mobile platform to the roll joint center, i.e. hr
    double ha_mm = 1;          // from the axis of rotation of the roll joint to the arm, i.e. hr
    double la_mm = 1;          // roll joint to the tilt joint, i.e. la
    double lt_mm = 1;          // tilt joint to the force sensor origin, i.e. lt
    double ht_mm = 1;          // tool from the tilt joint to the tool tip, i.e. ht
    double roll_rad = 1;       // offset to roll joint (0 for straight up, 180 degrees for upside down), i.e. theta_r
    double tool_rad = 1;       // offset from force sensor to center of tool axis, i.e. theta_t

};


#endif //GALEN_SIMULATION_AMBF_GALENSIMPMODEL_H
