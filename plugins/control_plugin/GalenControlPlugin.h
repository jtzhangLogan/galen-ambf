//
// Created by maxwe on 1/17/2022.
//

#ifndef GALEN_SIMULATION_AMBF_GALENCONTROLPLUGIN_H
#define GALEN_SIMULATION_AMBF_GALENCONTROLPLUGIN_H

#include "GalenInterface.h"
#include "GalenInvKin.h"
#include <afFramework.h>

using namespace ambf;

enum class ControlMode{
    GALEN_CONTROL, INPUT_DEVICE_CONTROL, DRAW_TOOL
};

class GalenControlPlugin: public afModelPlugin {

    virtual int init(const afModelPtr a_objectPtr, afModelAttribsPtr a_attribs) override;

    virtual void graphicsUpdate() override;
    virtual void physicsUpdate(double dt) override;

    virtual void reset() override;
    virtual bool close() override;

    void controlModeCheck(bool btn, double dt);

    //
    GalenInterface* galenInterface;

    // TODO: add inverse kinematics
    // GalenInvKin* galenInvKin;

    double m_switchModeTime = 1.5;
    double m_lastPressedTime = 0.0;
    double m_runningTime = 0.0;

    bool m_lastState = false;
    bool m_trigger = false;

    ControlMode controlMode;

    // joint pointers to each joint object
    int numJoints;
    int numPrismaticJoints = 3;
    int numRevoluteJoints = 2;
    vector<afJointPtr> joints;
    afJointPtr ATI;

    // TODO: why making them global on github?
    cGenericHapticDevicePtr g_hapticDevice;
    cHapticDeviceHandler* g_deviceHandler;
};

AF_REGISTER_MODEL_PLUGIN(GalenControlPlugin)

#endif //GALEN_SIMULATION_AMBF_GALENCONTROLPLUGIN_H
