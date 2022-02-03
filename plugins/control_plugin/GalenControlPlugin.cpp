//
// Created by maxwe on 1/17/2022.
//

#include "GalenControlPlugin.h"
#include <afConversions.h>

cMesh* arrow_ATI_nano_x;
cMesh* arrow_ATI_nano_y;
cMesh* arrow_ATI_nano_z;

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

    // initialize joint pointers
    joints.push_back(m_modelPtr->getJoint("carriage1_joint"));
    joints.push_back(m_modelPtr->getJoint("carriage2_joint"));
    joints.push_back(m_modelPtr->getJoint("carriage3_joint"));
    joints.push_back(m_modelPtr->getJoint("roll_joint"));
    joints.push_back(m_modelPtr->getJoint("tilt_joint"));

    numJoints = static_cast<int>(joints.size());

    std::cerr << numJoints << std::endl ;

    // initialize force vector and add to world plane
    arrow_ATI_nano_x = new cMesh();
    arrow_ATI_nano_y = new cMesh();
    arrow_ATI_nano_z = new cMesh();

    cCreateArrow(arrow_ATI_nano_x);
    cCreateArrow(arrow_ATI_nano_y);
    cCreateArrow(arrow_ATI_nano_z);

    m_modelPtr->getWorldPtr()->addSceneObjectToWorld(arrow_ATI_nano_x);
    m_modelPtr->getWorldPtr()->addSceneObjectToWorld(arrow_ATI_nano_y);
    m_modelPtr->getWorldPtr()->addSceneObjectToWorld(arrow_ATI_nano_z);

    ATI = m_modelPtr->getJoint("Tilt Distal Linkage and Force Sensor-Endoscope 35 degree in REMS");

    return 1;
}

int start_counter = 0;
int counter_toggle = 100;

void GalenControlPlugin::graphicsUpdate() {
    if (start_counter <= counter_toggle) {
        start_counter++;
    }

    std::cerr << ATI->getLocalPos() << std::endl;
    std::cerr << ATI->getLocalRot().getRow(0) << std::endl << ATI->getLocalRot().getRow(1) << std::endl << ATI->getLocalRot().getRow(2) << std::endl;

    arrow_ATI_nano_z->setLocalRot(ATI->getLocalRot());
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
            vector<double> measured_tra_jv = galenInterface->get_measured_tra_jv();
            vector<double> measured_rot_jv = galenInterface->get_measured_rot_jv();

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