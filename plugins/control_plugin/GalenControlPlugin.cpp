//
// Created by maxwe on 1/17/2022.
//

#include "GalenControlPlugin.h"
#include <afConversions.h>

cMesh* arrow_ATI_nano_x;
cMesh* arrow_ATI_nano_y;
cMesh* arrow_ATI_nano_z;

double map_joints(double x, double in_max, double in_min, double out_max, double out_min) {
    double  result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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

    controlMode = ControlMode::GALEN_CONTROL;

    cMatrix3d rot;
    rot.setExtrinsicEulerRotationDeg(180, 0, 90, C_EULER_ORDER_XYZ);

    rot.setExtrinsicEulerRotationDeg(0, -90, 0, C_EULER_ORDER_XYZ);
    // ----------------T_TipOffset.setLocalRot(rot);

    // initialize joint pointers
    joints.push_back(m_modelPtr->getJoint("carriage3_joint"));
    joints.push_back(m_modelPtr->getJoint("carriage1_joint"));
    joints.push_back(m_modelPtr->getJoint("carriage2_joint"));
    joints.push_back(m_modelPtr->getJoint("roll_joint"));
    //joints.push_back(m_modelPtr->getJoint("tilt_joint"));
    joints.push_back(m_modelPtr->getJoint("Tilt Middle Linkage-Tilt Distal Linkage and Force Sensor"));

    numJoints = static_cast<int>(joints.size());

    std::cerr << "number of joints " << numJoints << std::endl;
    /* Volumetric Drilling Init*/
    volumetricDrillingInit(m_worldPtr);

    /*SDF Init*/
    SDF_Init(m_worldPtr);
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
    m_drillRigidBody = m_modelPtr->getRigidBody("drill_tip");  
    if (!m_drillRigidBody){
        /*If not in world, try finding it in Model map*/
        m_drillRigidBody = m_worldPtr->getRigidBody("drill_tip");
        /*Exit if fail to find it*/
        if (!m_drillRigidBody){
            cerr << "ERROR! FAILED TO FIND DRILL RIGID BODY NAMED " << "mastoidectomy_drill" << endl;
            exit(EXIT_FAILURE);
        }
    }
    m_burrMesh = new cShapeSphere(0.043); // 2mm by default with 1 AMBF unit = 0.049664 m
    m_burrMesh->setRadius(0.001);
    m_burrMesh->m_material->setBlack();
    m_burrMesh->m_material->setShininess(0);
    m_burrMesh->m_material->m_specular.set(0, 0, 0);
    m_burrMesh->setShowEnabled(true);
    m_drillRigidBody->addChildSceneObject(m_burrMesh, cTransform());
    m_worldPtr->addSceneObjectToWorld(m_burrMesh);

    /*Get reference for volxel objetc*/
    while(!m_volumeObject){
        m_volumeObject = m_modelPtr->getVolume("mastoidectomy_volume_low");
        if(!m_volumeObject){
            m_volumeObject = m_worldPtr->getVolume("mastoidectomy_volume_low");
        }
        if (!m_volumeObject){
            cerr << "ERROR! FAILED TO FIND DRILL VOLUME NAMED " << "mastoidectomy_volume_low" << endl;
            return -1;
        }
        else{
            m_voxelObj = m_volumeObject->getInternalVolume();
        }
    }

    //Initialize tool cursors
    toolCursorInit();

    //Initialize Warning panels
    warningPanelInit();



}

void GalenControlPlugin::warningPanelInit(){
    // create a font
    cFontPtr font = NEW_CFONTCALIBRI40();

    // A warning pop-up that shows up while drilling at critical region
    m_warningPopup = new cPanel();
    m_warningPopup->set(m_mainCamera->m_width/2, m_mainCamera->m_height/5);
    m_warningPopup->setColor(cColorf(0.6,0,0));
    m_warningPopup->setLocalPos(m_mainCamera->m_width*0.3, m_mainCamera->m_height*0.6, 0);
    m_mainCamera->getFrontLayer()->addChild(m_warningPopup);
    m_warningPopup->setShowPanel(false);

    m_warningText = new cLabel(font);
    m_warningText->setLocalPos(0.31 * m_mainCamera->m_width, 0.67 * m_mainCamera->m_height, 0.5);
    m_warningText->m_fontColor.setWhite();
    m_warningText->setFontScale(1.0);
    m_warningText->setText("WARNING! Critical Region Detected");
    m_mainCamera->getFrontLayer()->addChild(m_warningText);
    m_warningText->setShowEnabled(false);
}

///
/// \brief This method initializes necessary components of SDF for force feedback
/// \param m_worldPtr    A world that contains all objects of the virtual environment
/// \return 1 if successful, 0 otherwise.
///
int GalenControlPlugin::SDF_Init(afWorldPtr m_worldPtr){
    /*Mark ballTester enabled flag*/
    ballTesterEnabled = true;
    /*Initialize tester ball*/
    testerBall = new cShapeSphere(0.1);
    testerBall->setRadius(0.05);
    testerBall->m_material->setRed();
    testerBall->m_material->setShininess(0);
    testerBall->m_material->m_specular.set(0, 0, 0);
    testerBall->setShowEnabled(true);
    /*Set location of ball to center of mass of bone volume*/
    cMatrix3d voxelObjRot = m_voxelObj -> getLocalRot();
    cVector3d voxelObjPos = m_volumeObject->getLocalPos();
    cVector3d volumeCenterofMass = voxelObjRot* ((m_voxelObj->m_minCorner + m_voxelObj->m_maxCorner)/2) + voxelObjPos;
    testerBall->setLocalPos( volumeCenterofMass);
    m_worldPtr->addSceneObjectToWorld(testerBall);

    /*Initialize A Text Pannel To Display SDF Distance Between Tip And closest critical region*/
    // A panel to display current drill size
    cFontPtr font = NEW_CFONTCALIBRI40();
    SDF_vectorDistancePanel = new cPanel();
    SDF_vectorDistancePanel->setSize(170, 50);
    SDF_vectorDistancePanel->setCornerRadius(10, 10, 10, 10);
    SDF_vectorDistancePanel->setLocalPos(40,60);
    SDF_vectorDistancePanel->setColor(cColorf(1, 1, 1));
    SDF_vectorDistancePanel->setTransparencyLevel(0.8);
    m_mainCamera->getFrontLayer()->addChild(SDF_vectorDistancePanel);
    ballTesetrDistanceText = new cLabel(font);
    ballTesetrDistanceText->setLocalPos(50,70);
    ballTesetrDistanceText->m_fontColor.setBlack();
    ballTesetrDistanceText->setFontScale(.5);
    ballTesetrDistanceText->setText("SDF Distance Vec:  x mm");
    m_mainCamera->getFrontLayer()->addChild(ballTesetrDistanceText);

    /*Initialize an arrow mesh to represent SDF distance vector*/
    SDF_distanceVectorMesh = new cMesh();
    SDF_distanceVectorMesh -> m_material ->setColorf( 0,0,1 );
    m_worldPtr -> addSceneObjectToWorld(SDF_distanceVectorMesh);

    return 1;
}

///
/// \brief This method calculates  the euclidean distance between the tester ball suface and the tool tip
/// \return the distance between the tester ball's surface and the tool tip
///TODO: This  is now refering the Endoscopic tool tip on the galen robot instead of the actual drill. Needs to be changed after drill integration
cVector3d GalenControlPlugin::getDistanceFromBallToTip(){
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
    cVector3d dist = (burrPos-ballLocation);
    //Scale to distance
    double length = dist.length();
    //Check if within ball
    double mag = length-testerBall->getRadius();
    mag = max(mag,0.001);
    dist = dist/length*(mag);
    return 1000*dist; //units in mm
}

///
/// \brief This method contains the service routine for the SDF publishing. This method should be invoked in physics update
/// \return void
void GalenControlPlugin::SDF_ServiceRoutine(){
    //Get distance from tip to tester Ball
    cVector3d dist = getDistanceFromBallToTip();
    galenInterface->pub_distance(dist);

    //Change display text
    if(ballTesetrDistanceText){
        ballTesetrDistanceText->setText("SDF Distance Vec:  \n"+ (dist).str()+" mm");
    }

    //Render an arrow to represent the SDF vector
    SDF_distanceVectorMesh -> clear();
    cVector3d arrowStartingPos = testerBall->getLocalPos()+ dist/(dist.length())*testerBall->getRadius();
    double R = min( 1.0 , 20 / (dist.length()) );
    cColorf arrowColor(  R ,1,0,0.3);
    cCreateArrow(SDF_distanceVectorMesh, dist.length()/1000 ,0.003,0.05,0.006,false,32,dist, arrowStartingPos,arrowColor );
    SDF_distanceVectorMesh ->m_material->setColorf( R, 1, 0, 0.3);
}

///
/// \brief This method updates the position of the shaft tool cursors which eventually updates the position of the whole tool.
/// \return void
void GalenControlPlugin::toolCursorPoseUpdate(cVector3d &pos){
    m_toolCursorList[0] -> setLocalPos(pos);
    m_toolCursorList[0] -> setDeviceGlobalPos(pos);
}


///
/// \brief This method contains the service routine for thevolumetric drilling algorithm. This method should be invoked in physics update
/// \return void
void GalenControlPlugin::volumetricDrillingServiceRoutine(){
    m_worldPtr->getChaiWorld()->computeGlobalPositions(true);
    //update tool cursor pos
    cVector3d toolPos = m_burrMesh->getLocalPos();
    
    toolCursorPoseUpdate(toolPos);
    //Voxel removing
    if (m_toolCursorList[0]->isInContact(m_voxelObj)){
        cout<<"DEBUG 1"<< endl;
        for (int ci = 0 ; ci < 3 ; ci++){
            // retrieve contact event
            cCollisionEvent* contact = m_toolCursorList[0]->m_hapticPoint->getCollisionEvent(ci);

            cVector3d orig(contact->m_voxelIndexX, contact->m_voxelIndexY, contact->m_voxelIndexZ);

            m_voxelObj->m_texture->m_image->getVoxelColor(uint(orig.x()), uint(orig.y()), uint(orig.z()), m_storedColor);

            //if the tool comes in contact with the critical region, instantiate the warning message
            if(m_storedColor != m_boneColor && m_storedColor != m_zeroColor)
            {
                m_warningPopup->setShowPanel(true);
                m_warningText->setShowEnabled(true);
            }

            m_voxelObj->m_texture->m_image->setVoxelColor(uint(orig.x()), uint(orig.y()), uint(orig.z()), m_zeroColor);

            //Publisher for voxels removed
            if(m_storedColor != m_zeroColor)
            {
                double sim_time = m_drillRigidBody->getCurrentTimeStamp();

                double voxel_array[3] = {orig.get(0), orig.get(1), orig.get(2)};

                cColorf color_glFloat = m_storedColor.getColorf();
                float color_array[4];
                color_array[0] = color_glFloat.getR();
                color_array[1] = color_glFloat.getG();
                color_array[2] = color_glFloat.getB();
                color_array[3] = color_glFloat.getA();

                /*TODO: is drilling pub necessary*/
                //m_drillingPub -> voxelsRemoved(voxel_array,color_array,sim_time);
            }

            m_mutexVoxel.acquire();
            m_volumeUpdate.enclose(cVector3d(uint(orig.x()), uint(orig.y()), uint(orig.z())));
            m_mutexVoxel.release();
            
        }
        // mark voxel for update
        m_flagMarkVolumeForUpdate = true;
    }
    // remove warning panel
    else
    {
        m_warningPopup->setShowPanel(false);
        m_warningText->setShowEnabled(false);
    }
    // compute interaction forces
    for(int i = 0 ; i < m_toolCursorList.size() ; i++){
        m_toolCursorList[i]->computeInteractionForces();
    }
}

///
/// \brief This method contains the additional service routine for thevolumetric drilling algorithm. This method should be invoked in graphics update
/// \return void
void GalenControlPlugin::volumetricDrillingServiceRoutine_Graphics(){
        // update region of voxels to be updated
    if (m_flagMarkVolumeForUpdate)
    {
        m_mutexVoxel.acquire();
        cVector3d min = m_volumeUpdate.m_min;
        cVector3d max = m_volumeUpdate.m_max;
        m_volumeUpdate.setEmpty();
        m_mutexVoxel.release();
        ((cTexture3d*)m_voxelObj->m_texture.get())->markForPartialUpdate(min, max);
        m_flagMarkVolumeForUpdate = false;
    }
}

///
/// \brief This method contains the initialization of tool cursors in the af world
/// \return void
void GalenControlPlugin::toolCursorInit(){
    m_toolCursorList.resize(1);                                 //TODO: currently only one tool cursor, may add more to 8 later
    for (int i = 0; i < m_toolCursorList.size() ; i++ ){
        m_toolCursorList[i] = new cToolCursor(m_worldPtr -> getChaiWorld());
        m_worldPtr->addSceneObjectToWorld(m_toolCursorList[i]);

        if(i == 0){
            //m_toolCursorList[i] -> setHapticDevice(m_hapticDevice);
            // map the physical workspace of the haptic device to a larger virtual workspace.

            m_toolCursorList[i]->setWorkspaceRadius(10.0);
            //m_toolCursorList[i]->setWaitForSmallForce(true);
            cout<< "DEBUG" << m_toolCursorList[i]->start() <<endl;;
            m_toolCursorList[i]->m_hapticPoint->m_sphereProxy->setShowFrame(false);
            m_toolCursorList[i]-> setRadius(0.0043);
            m_toolCursorList[i]->m_name = "mastoidectomy_drill";
            //This method sets the display options of the goal and proxy spheres. If both spheres are enabled, a small line is drawn between both spheres.
            m_toolCursorList[i]->m_hapticPoint->setShow(false, false); 
            m_toolCursorList[i]->m_hapticPoint->m_sphereProxy->m_material->setRedCrimson();
            m_toolCursorList[i]->m_hapticPoint->m_sphereGoal->m_material->setBlueAquamarine();
        }
        else{
            /*
            m_toolCursorList[i]->setShowContactPoints(m_showGoalProxySpheres, m_showGoalProxySpheres);
             m_toolCursorList[i]->setRadius(m_toolCursorRadius[i]);
            */
            m_toolCursorList[i]->m_hapticPoint->m_sphereProxy->m_material->setGreenChartreuse();
            m_toolCursorList[i]->m_hapticPoint->m_sphereGoal->m_material->setOrangeCoral();
           
        }

    }

    cVector3d toolPos = m_burrMesh->getLocalPos();
    toolCursorPoseUpdate(toolPos);
    for(int i = 0; i< m_toolCursorList.size(); i++){
        m_toolCursorList[i] -> initialize();
    }
    
}

int start_counter = 0;
int counter_toggle = 100;

void GalenControlPlugin::graphicsUpdate() {
    if (start_counter <= counter_toggle) {
        start_counter++;
    }

    //voxel removing
    volumetricDrillingServiceRoutine_Graphics(); 

}

void GalenControlPlugin::physicsUpdate(double dt){
    /*
     * update mesh
     */

    if (start_counter < 10){
        return;
    }

    // update physics based on control mode
    switch (controlMode) {

        case ControlMode::GALEN_CONTROL:{
            /*
             * follow real world robot movement
             */

            vector<double> measured_jp = galenInterface->get_measured_jp();
            if(!measured_jp.empty()){
              for (int idx = 0 ; idx < numJoints ; idx++){
                  double initialPos = map_joints(measured_jp[idx], physical_joint_limits_upper[idx], physical_joint_limits_lower[idx],
                                                 joints[idx]->getUpperLimit(), joints[idx]->getLowerLimit());

                  //joints[idx]->commandPosition(measured_jp[idx]);
                  joints[idx]->commandPosition(initialPos);
              }
            }


        }
        break;

        case ControlMode::INPUT_DEVICE_CONTROL:{
            /*
             * input device commands desired cartisan position
             */
            double value = -1.0;
            m_modelPtr->getJoint("tilt_joint")->commandPosition(value);
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

    /*Service Routines Methods*/
    SDF_ServiceRoutine();
    volumetricDrillingServiceRoutine();

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
    delete SDF_distanceVectorMesh;
    delete SDF_vectorDistancePanel;
    delete ballTesetrDistanceText;
    return 0;
}

void GalenControlPlugin::reset(){}
