//
// Created by maxwe on 1/17/2022.
//

#include "GalenControlPlugin.h"
#include <afConversions.h>

cMesh* arrow_ATI_nano_x;
cMesh* arrow_ATI_nano_y;
cMesh* arrow_ATI_nano_z;

double map_joints(double x, double in_max, double in_min, double out_max, double out_min) {
    //std::cerr << x << "---" << in_max << "---" << in_min << "---" << out_max << "---" << out_min << "---" << std::endl;
    double  result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    //std::cerr << result << std::endl;
    return result;
}

int GalenControlPlugin::init(const afModelPtr a_modelPtr, afModelAttribsPtr a_attribs){
    /*
     * naive initialization ignoring input device
     */

    /*Get World Pointer*/
    m_modelPtr = a_modelPtr;
    m_worldPtr = m_modelPtr->getWorldPtr();     //Ptr to the simulation world

    /*Get Main Camera*/
    m_mainCamera = m_worldPtr->getCameras()[0];
    if(!m_mainCamera){
        m_mainCamera = m_modelPtr->getCameras()[0];
    }


    galenInterface = new GalenInterface();

    // galenInvKin = new GalenInvKin;

    // TODO: switch back to Galen mode
    // controlMode = ControlMode::GALEN_CONTROL;
    controlMode = ControlMode::GALEN_CONTROL;

    /*===============================================HongYi Fan ================================================*/
    controlMode = ControlMode::NO_CONTROL;
     /*==========================================END HongYi Fan ================================================*/


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
    joints.push_back(m_modelPtr->getJoint("carriage3_joint"));
    joints.push_back(m_modelPtr->getJoint("carriage1_joint"));
    joints.push_back(m_modelPtr->getJoint("carriage2_joint"));
    joints.push_back(m_modelPtr->getJoint("roll_joint"));
    joints.push_back(m_modelPtr->getJoint("tilt_joint"));

    numJoints = static_cast<int>(joints.size());

    std::cerr << "number of joints " << numJoints << std::endl;

    // initialize force vector and add to world plane
    /*
    arrow_ATI_nano_x = new cMesh();
    arrow_ATI_nano_y = new cMesh();
    arrow_ATI_nano_z = new cMesh();

    cCreateArrow(arrow_ATI_nano_x);
    cCreateArrow(arrow_ATI_nano_y);
    cCreateArrow(arrow_ATI_nano_z);

    m_modelPtr->getWorldPtr()->addSceneObjectToWorld(arrow_ATI_nano_x);
    m_modelPtr->getWorldPtr()->addSceneObjectToWorld(arrow_ATI_nano_y);
    m_modelPtr->getWorldPtr()->addSceneObjectToWorld(arrow_ATI_nano_z); */

    // ATI = m_modelPtr->getJoint("Tilt Distal Linkage and Force Sensor-Endoscope 35 degree in REMS");


    /*================================HongYi Fan=====================================*/
    /* Volumetric Drilling Init*/
    volumetricDrillingInit(m_worldPtr);

    /*ballTesterInit*/
    ballTesterInit(m_worldPtr);

    /*============================End HongYi Fan=====================================*/
    return 1;
}

///
/// \brief This method initializes necessary components of volumetric drilling function
/// \param m_worldPtr    A world that contains all objects of the virtual environment
/// \return 1 if successful, 0 otherwise.
///
int GalenControlPlugin::volumetricDrillingInit(afWorldPtr m_worldPtr){

    //Color of voxels
    m_zeroColor = cColorb(0x00, 0x00, 0x00, 0x00);
    m_boneColor = cColorb(255, 249, 219, 255);
    m_storedColor = cColorb(0x00, 0x00, 0x00, 0x00);

    //Setup drill model and voxel object
    /*Find drill in world map*/
    m_drillRigidBody = m_worldPtr->getRigidBody("Endoscope Tip");  //TODO: this is refering to the endoscopic tip, need to be changed back to drill model
    if (!m_drillRigidBody){
        /*If not in world, try finding it in Model map*/
        m_drillRigidBody = m_modelPtr->getRigidBody("Endoscope Tip"); 
        /*Exit if fail to find it*/
        if (!m_drillRigidBody){
            cerr << "ERROR! FAILED TO FIND DRILL RIGID BODY NAMED " << "Endoscope Tip" << endl;
            exit(EXIT_FAILURE);
        }
    }
    m_burrMesh = new cShapeSphere(0.043); // 2mm by default with 1 AMBF unit = 0.049664 m
    m_burrMesh->setRadius(0.043);
    m_burrMesh->m_material->setBlack();
    m_burrMesh->m_material->setShininess(0);
    m_burrMesh->m_material->m_specular.set(0, 0, 0);
    m_burrMesh->setShowEnabled(true);
    m_drillRigidBody->addChildSceneObject(m_burrMesh, cTransform());
    m_worldPtr->addSceneObjectToWorld(m_burrMesh);
    

    m_volumeObject = m_worldPtr->getVolume("mastoidectomy_volume");
    if (!m_volumeObject){
        cerr << "ERROR! FAILED TO FIND DRILL VOLUME NAMED " << "mastoidectomy_volume" << endl;
        return -1;
    }
    else{
        m_voxelObj = m_volumeObject->getInternalVolume();
    }

}

///
/// \brief This method initializes necessary components of ball tester for force feedback 
/// \param m_worldPtr    A world that contains all objects of the virtual environment
/// \return 1 if successful, 0 otherwise.
///
int GalenControlPlugin::ballTesterInit(afWorldPtr m_worldPtr){
    /*Mark ballTester enabled flag*/
    ballTesterEnabled = true;
    /*Initialize tester ball*/
    testerBall = new cShapeSphere(0.1);
    testerBall->setRadius(0.1);
    testerBall->m_material->setRed();
    testerBall->m_material->setShininess(0);
    testerBall->m_material->m_specular.set(0, 0, 0);
    testerBall->setShowEnabled(true);
    testerBall->setLocalPos( cVector3d(0.0,0.0,0.0) );
    m_worldPtr->addSceneObjectToWorld(testerBall);
    
    /*Initialize A Text Pannel To Display Distance Between Tip And Ball*/
    // A panel to display current drill size
    cFontPtr font = NEW_CFONTCALIBRI40();
    ballTesterDistancePanel = new cPanel();
    ballTesterDistancePanel->setSize(170, 50);
    ballTesterDistancePanel->setCornerRadius(10, 10, 10, 10);
    ballTesterDistancePanel->setLocalPos(40,60);
    ballTesterDistancePanel->setColor(cColorf(1, 1, 1));
    ballTesterDistancePanel->setTransparencyLevel(0.8);
    m_mainCamera->getFrontLayer()->addChild(ballTesterDistancePanel);
    ballTesetrDistanceText = new cLabel(font);
    ballTesetrDistanceText->setLocalPos(50,70);
    ballTesetrDistanceText->m_fontColor.setBlack();
    ballTesetrDistanceText->setFontScale(.5);
    ballTesetrDistanceText->setText("Distance to Tester Ball:  x mm");
    m_mainCamera->getFrontLayer()->addChild(ballTesetrDistanceText);

    return 1;
}

///
/// \brief This method calculates  the euclidean distance between the tester ball and the tool tip
/// \return the distance between the tester ball's surface and the tool tip 
///TODO: This  is now refering the Endoscopic tool tip on the galen robot instead of the actual drill. Needs to be changed after drill integration
cVector3d GalenControlPlugin::getDistanceFromTipToBall(){
    //Check if testerBall is in use
    if(! ballTesterEnabled){
        cerr<<"ERROR: getDistanceFromTipToBall: Ball Tester is NOT enabled."<<endl;
        return 0;
    }

    //Get ball's location
    cVector3d ballLocation = testerBall->getLocalPos();
    //Get ball's radius
    double r = testerBall->getRadius();
    //Get tool tip location
    cVector3d burrPos = m_burrMesh->getLocalPos();
    //Return distance
    cVector3d dist = (ballLocation-burrPos);
    return dist;
}

///
/// \brief This method contains the service routine for the ball tester function. This method should be invoked in physics update
/// \return void
void GalenControlPlugin::ballTesterServiceRoutine(){
    //Get distance from tip to tester Ball
    cVector3d dist = getDistanceFromTipToBall();
    //Change display text
    if(ballTesetrDistanceText){
        ballTesetrDistanceText->setText("Distance Vec to Ball:  \n"+ dist.str()+" mm");
    }
    
}

int start_counter = 0;
int counter_toggle = 100;

void GalenControlPlugin::graphicsUpdate() {
    if (start_counter <= counter_toggle) {
        start_counter++;
    }

    //std::cerr << ATI->getLocalPos() << std::endl;
    //std::cerr << ATI->getLocalRot().getRow(0) << std::endl << ATI->getLocalRot().getRow(1) << std::endl << ATI->getLocalRot().getRow(2) << std::endl;

    //arrow_ATI_nano_z->setLocalRot(ATI->getLocalRot());
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
        // move to initial position
        vector<double> measured_jp = galenInterface->get_measured_jp();

        for (int idx = 0 ; idx < numJoints ; idx++){
            double initialPos = map_joints(measured_jp[idx], physical_joint_limits_upper[idx], physical_joint_limits_lower[idx],
                                           joints[idx]->getUpperLimit(), joints[idx]->getLowerLimit());
            joints[idx]->commandPosition(measured_jp[idx]);
        }

        // TODO: why this part?
        // m_psmIK.m_FK->computeFK(measured_jp, 4, T_7_0);
        return;
    }*/

    // update physics based on control mode
    switch (controlMode) {

        case ControlMode::GALEN_CONTROL:{
            /*
             * follow real world robot movement
             */
            /*
            // get mesasured joint states from real world robot via ros
            vector<double> measured_jv = galenInterface->get_measured_jv();

            // vector<double> measured_tra_jv = galenInterface->get_measured_tra_jv();
            // vector<double> measured_rot_jv = galenInterface->get_measured_rot_jv();

            // set simulation joint angle
            // TODO: need to add new logic for commandPosition?

            for (int idx = 0 ; idx < numJoints ; idx++){
                // move at actual joint velocity
                double commandVelocity = map_joints(measured_jv[idx], physical_joint_limits_upper[idx], physical_joint_limits_lower[idx],
                                                    joints[idx]->getUpperLimit(), joints[idx]->getLowerLimit());
                std::cerr << commandVelocity << std::endl;
                joints[idx]->commandVelocity(commandVelocity);
            }*/
            std::cerr << "position" << std::endl;
            vector<double> measured_jp = galenInterface->get_measured_jp();

            for (int idx = 0 ; idx < numJoints ; idx++){
                double initialPos = map_joints(measured_jp[idx], physical_joint_limits_upper[idx], physical_joint_limits_lower[idx],
                                               joints[idx]->getUpperLimit(), joints[idx]->getLowerLimit());
                joints[idx]->commandPosition(measured_jp[idx]);
            }

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
            ballTesterServiceRoutine();


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
