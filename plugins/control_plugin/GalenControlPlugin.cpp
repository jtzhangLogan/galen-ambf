//
// Created by maxwe on 1/17/2022.
//

#include "GalenControlPlugin.h"
#include <afConversions.h>

cMesh* arrow_ATI_nano_x;
cMesh* arrow_ATI_nano_y;
cMesh* arrow_ATI_nano_z;

double map_general(double x, double in_max, double in_min, double out_max, double out_min) {
    //std::cerr << x << "---" << in_max << "---" << in_min << "---" << out_max << "---" << out_min << "---" << std::endl;
    double  result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    //std::cerr << result << std::endl;
    return result;
}

int GalenControlPlugin::init(const afModelPtr a_modelPtr, afModelAttribsPtr a_attribs){
    /*
     * naive initialization ignoring input device
     */

    m_modelPtr = a_modelPtr;

    galenInterface = new GalenInterface();

    // galenInvKin = new GalenInvKin;

    // TODO: switch back to Galen mode
    // controlMode = ControlMode::GALEN_CONTROL;
    controlMode = ControlMode::GALEN_CONTROL;

    // Initialize Input Device
    /*
    g_deviceHandler = new cHapticDeviceHandler();
    if (!g_deviceHandler->getDevice(g_hapticDevice, 0)){
        return -1;
    }
    g_hapticDevice->open();
    cerr << "FOUND DEVICE " << g_hapticDevice->getSpecifications().m_manufacturerName << ", " << g_hapticDevice->getSpecifications().m_modelName << endl;
    */

    // TODO: what does this do?
    cMatrix3d rot;
    rot.setExtrinsicEulerRotationDeg(180, 0, 90, C_EULER_ORDER_XYZ);
    // ----------------T_7_0.setLocalRot(rot);
    // ----------------T_7_0.setLocalPos(cVector3d(0, 0, -0.3));

    // find base
    // TODO: whats the purpose? temporarily commented out cause of error

    /*
    afRigidBodyPtr baseLink = m_modelPtr->getRigidBody("Carriage1");
    if (!baseLink){
        cerr << "ERROR! Base link NOT FOUND " << endl;
        return -1;
    } */
    // ----------------T_BaseOffset << baseLink->getCOMTransform();

    // TODO: what does this do?
    rot.setExtrinsicEulerRotationDeg(0, -90, 0, C_EULER_ORDER_XYZ);
    // ----------------T_TipOffset.setLocalRot(rot);

    // -----------------------------------------------
    // initialize joint pointers
    // -----------------------------------------------
    joints.push_back(m_modelPtr->getJoint("carriage1_joint"));
    joints.push_back(m_modelPtr->getJoint("carriage2_joint"));
    joints.push_back(m_modelPtr->getJoint("carriage3_joint"));
    joints.push_back(m_modelPtr->getJoint("roll_joint"));
    joints.push_back(m_modelPtr->getJoint("tilt_joint"));

    numJoints = static_cast<int>(joints.size());

    std::cerr << numJoints << std::endl ;

    // -----------------------------------------------
    // initialize force vector and add to world plane
    // -----------------------------------------------
    arrow_ATI_nano_x = new cMesh();
    arrow_ATI_nano_y = new cMesh();
    arrow_ATI_nano_z = new cMesh();

    cCreateArrow(arrow_ATI_nano_x, 1, 0.01, 0.05, 0.015,
                 false, 32, cVector3d(1,0,0), cVector3d(0,0,0));

    cCreateArrow(arrow_ATI_nano_y,  1, 0.01, 0.05, 0.015,
                 false, 32, cVector3d(0,0,1), cVector3d(0,0,0));

    cCreateArrow(arrow_ATI_nano_z, 1, 0.01, 0.05, 0.015,
                 false, 32, cVector3d(0, 1, 0), cVector3d(0,0,0));

    // change color
    cColorf color_x, color_y, color_z;
    color_x.setBlue();
    color_y.setGreen();
    color_z.setRed();
    arrow_ATI_nano_x->m_material->setColor(color_x);
    arrow_ATI_nano_y->m_material->setColor(color_y);
    arrow_ATI_nano_z->m_material->setColor(color_z);

    m_modelPtr->getWorldPtr()->addSceneObjectToWorld(arrow_ATI_nano_x);
    m_modelPtr->getWorldPtr()->addSceneObjectToWorld(arrow_ATI_nano_y);
    m_modelPtr->getWorldPtr()->addSceneObjectToWorld(arrow_ATI_nano_z);

    // -----------------------------------------------
    // find ATI and carriage body
    // -----------------------------------------------
    bodys.push_back(m_modelPtr->getRigidBody("Tilt Distal Linkage and Force Sensor"));
    bodys.push_back(m_modelPtr->getRigidBody("Carriage1"));
    bodys.push_back(m_modelPtr->getRigidBody("Carriage2"));
    bodys.push_back(m_modelPtr->getRigidBody("Carriage3"));
    std::cerr << bodys.size() << std::endl;

    rot_ati_ambf.setCol0(cVector3d(1, 0, 0));
    rot_ati_ambf.setCol1(cVector3d(0, -1, 0));
    rot_ati_ambf.setCol2(cVector3d(0, 0, 1));
    return 1;
}

int start_counter = 0;
int counter_toggle = 100;

void GalenControlPlugin::graphicsUpdate() {
    if (start_counter <= counter_toggle) {
        start_counter++;
    }

    // ------------------------------------------------
    // Update Carriage color
    // ------------------------------------------------


    // ------------------------------------------------
    // Update ATI force arrow
    // ------------------------------------------------
    // find ATI position, add an offset to the arrow
    auto offset = cVector3d(0.1, 0.1, 0.1);

    // change arrow length according to measured force magnitude
    vector<double> measured_cf = galenInterface->get_measured_cf();

    double a_length_x = 0;
    double a_length_y = 0;
    double a_length_z = 0;

    if (measured_cf.size() != 0) {
        a_length_x = measured_cf[0];
        a_length_y = measured_cf[1];
        a_length_z = measured_cf[2];
    }

    // map from force magnitude to length, now we clamp the length between 0.1 to 1.0
    double a_length_x_sim, a_length_y_sim, a_length_z_sim;
    a_length_x_sim = map_general(a_length_x, 20, -20, 1.0, -1.0);
    a_length_y_sim = map_general(a_length_y, 20, -20, 1.0, -1.0);
    a_length_z_sim = map_general(a_length_z, 20, -20, 1.0, -1.0);

    // update z
    arrow_ATI_nano_z->scaleXYZ(1, std::abs(a_length_z_sim)+0.1, 1);
    if (a_length_z_sim > 0) {
        arrow_ATI_nano_z->setLocalRot(ATI->getLocalRot());
    } else {
        arrow_ATI_nano_z->setLocalRot(ATI->getLocalRot() * rot_ati_ambf);
    }
    arrow_ATI_nano_z->setLocalPos(ATI->getLocalPos() + offset);

    // update y
    arrow_ATI_nano_y->scaleXYZ(1, 1, std::abs(a_length_y_sim)+0.1);
    if (a_length_y_sim > 0) {
        arrow_ATI_nano_y->setLocalRot(ATI->getLocalRot());
    } else {
        cMatrix3d rot;
        rot.setCol0(cVector3d(1, 0, 0));
        rot.setCol1(cVector3d(0, 1, 0));
        rot.setCol2(cVector3d(0, 0, -1));
        arrow_ATI_nano_y->setLocalRot(ATI->getLocalRot() * rot);
    }
    arrow_ATI_nano_y->setLocalPos(ATI->getLocalPos() + offset);

    // update x
    arrow_ATI_nano_x->scaleXYZ(std::abs(a_length_x_sim)+0.1, 1, 1);
    if (a_length_x_sim < 0) {
        arrow_ATI_nano_x->setLocalRot(ATI->getLocalRot());
    } else {
        cMatrix3d rot;
        rot.setCol0(cVector3d(-1, 0, 0));
        rot.setCol1(cVector3d(0, 1, 0));
        rot.setCol2(cVector3d(0, 0, 1));
        arrow_ATI_nano_x->setLocalRot(ATI->getLocalRot() * rot);
    }
    arrow_ATI_nano_x->setLocalPos(ATI->getLocalPos() + offset);

}

void GalenControlPlugin::physicsUpdate(double dt){
    /*
     * update mesh
     */

    if (start_counter < 10){
        return;
    }
    /*
    else if (start_counter < counter_toggle){
        // TODO: add inverse kinematic

        vector<double> measured_jp = galenInterface->get_measured_jp();

        measured_jp[2] *= 10.; // Compensate for 2nd joint

        for (int idx = 0 ; idx < numJoints ; idx++){
            joints[idx]->commandPosition(measured_jp[idx]);
        }
        // TODO: why this part?
        m_psmIK.m_FK->computeFK(measured_jp, 4, T_7_0);

        return;
    } */

    // update physics based on control mode
    switch (controlMode) {

        case ControlMode::GALEN_CONTROL:{
            /*
             * follow real world robot movement
             */

            // get mesasured joint states from real world robot via ros
            vector<double> measured_jp = galenInterface->get_measured_jp();

            if (measured_jp.size() != 0) {
                // physical-sim: 1->3, 2->1, 3->2
                for (int idx = 0 ; idx < numJoints ; idx++){
                    double cmd = map_general(measured_jp[idx], physical_joint_limits_upper[idx], physical_joint_limits_lower[idx],
                                             joints[idx]->getUpperLimit(), joints[idx]->getLowerLimit());
                    joints[idx]->commandPosition(cmd);
                }
            }

            // vector<double> measured_tra_jv = galenInterface->get_measured_tra_jv();
            // vector<double> measured_rot_jv = galenInterface->get_measured_rot_jv();

            // set simulation joint angle
            // TODO: need to add new logic for commandPosition?
            /*
            for (int idx = 0 ; idx < numPrismaticJoints ; idx++){
                // move at actual joint velocity
                joints[idx]->commandVelocity(measured_tra_jv[idx]);
            }
            for (int idx = 0 ; idx < numRevoluteJoints ; idx++){
                // move at actual joint velocity
                joints[idx+numPrismaticJoints]->commandVelocity(measured_rot_jv[idx]);
            }*/
        }
        break;

        case ControlMode::INPUT_DEVICE_CONTROL:{
            /*
             * input device commands desired cartisan position
             */
            double value = -1.0;
            m_modelPtr->getJoint("tilt_joint")->commandPosition(value);

            // TODO: remove this after testing, just for verifying math equations here
            //galenInvKin->_compute_inv_kin(galenInterface->get_tool_cp(), galenInterface->get_measured_jp());

            /*
            for (auto c : galenInterface->get_measured_jp()) {
                std::cerr << c << std::endl;
            } */

            break;

            // get input device commanded position
            cTransform devPose;
            cVector3d  linVel;
            g_hapticDevice->getTransform(devPose);
            g_hapticDevice->getLinearVelocity(linVel);

            // check if user is using device
            bool pressed;
            g_hapticDevice->getUserSwitch(1, pressed);
            if (pressed){
                linVel.set(0, 0, 0);
            }

            // TODO: what does these do?
            /*
            cVector3d posCmd = T_7_0.getLocalPos() + cTranspose(T_BaseOffset.getLocalRot()) * linVel * dt;
            cMatrix3d rotCmd = cTranspose(T_BaseOffset.getLocalRot()) * devPose.getLocalRot() * T_TipOffset.getLocalRot();
            cTransform T_cmd(posCmd, rotCmd);
            T_7_0.setLocalPos(posCmd); */

            // computer inverse kinematics
            // galenInvKin->_compute_inv_kin(galenInterface->get_tool_cp(), galenInterface->get_measured_jp());

            // move simluation robot to desired position using inverse kinematics result
            vector<double> measured_jp = galenInterface->get_measured_jp();
            for (int idx = 0 ; idx < numJoints ; idx++){
                joints[idx]->commandPosition(measured_jp[idx]);
            }

            // move actual robot
            galenInterface->_servo_jp(measured_jp);
        }
            break;

        default:
            break;
    }

}

void GalenControlPlugin::controlModeCheck(bool btn, double dt){
    m_runningTime += dt;

    double timeDiff = 10.0;

    if (btn && !m_lastState){
        cerr << "Clutch Pressed:" << endl;
        m_trigger = true;
        timeDiff = m_runningTime - m_lastPressedTime;
        m_lastPressedTime = m_runningTime;
        if (timeDiff <= m_switchModeTime){
            controlMode = static_cast<ControlMode>((static_cast<int>(controlMode) + 1) % (static_cast<int>(ControlMode::DRAW_TOOL) + 1));
            cerr << "Changing Control Mode: " << static_cast<int>(controlMode) << endl;
        }
    }

    m_trigger = false;
    m_lastState = btn;
}

bool GalenControlPlugin::close()
{
    delete galenInterface;
    return 0;
}

void GalenControlPlugin::reset(){}
