/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

#include <HFD/HFDDriver.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/core/objectmodel/ScriptEvent.h>

#include <sofa/core/visual/VisualParams.h>
#include <HFD/HFDVisualModel.h>
#include <thread>
#include <chrono>

namespace sofa::component::controller
{
    
using namespace sofa::defaulttype;
using namespace sofa::type;

unsigned int catchHDError(bool logError = true) { return 0x0000; }


// Callback method to be executed by HD scheduler to copy the data from haptic own struct @sa m_hapticData to SOFA one @sa m_simuData.
unsigned int __stdcall copyDeviceDataCallback(void * pUserData)
{
    HFDDriver * driver = (HFDDriver *)pUserData;
    driver->m_simuData = driver->m_hapticData;
    return 1;
}


// Callback method to get the tool position and angles and compute the Force to apply to the tool
void* __stdcall stateCallback(void* userData)
{
    HFDDriver* driver = (HFDDriver*)userData;

    driver->m_bThreadFinish = false;

    while (!driver->m_bThreadFinish)
    {
        if (driver->m_bOpen && driver->m_bInit && driver->m_bCalibrate)
        {
            hfdGetPosition(&(driver->m_Pos[0]), &(driver->m_Pos[1]), &(driver->m_Pos[2]));

            hfdGetOrientationFrame(driver->m_Orientation);

            hfdGetDeltaJointAngles(&(driver->m_BaseAngle[0]), &(driver->m_BaseAngle[1]),
                                    &(driver->m_BaseAngle[2]));

            hfdGetWristJointAngles(&(driver->m_ToolAngle[0]), &(driver->m_ToolAngle[1]),
                                    &(driver->m_ToolAngle[2]));
            
            Eigen::Matrix3d n_Ori;

            n_Ori << driver->m_Orientation[0][0], driver->m_Orientation[0][1], driver->m_Orientation[0][2], 
                     driver->m_Orientation[1][0], driver->m_Orientation[1][1], driver->m_Orientation[1][2], 
                     driver->m_Orientation[2][0], driver->m_Orientation[2][1], driver->m_Orientation[2][2];

            Eigen::Vector3d eulerAngle = n_Ori.eulerAngles(2, 1, 0);

            Eigen::AngleAxisd roll(Eigen::AngleAxisd(-eulerAngle(0), Eigen::Vector3d::UnitY()));
            Eigen::AngleAxisd pitch(Eigen::AngleAxisd(-eulerAngle(1), Eigen::Vector3d::UnitX()));
            Eigen::AngleAxisd yaw(Eigen::AngleAxisd(-eulerAngle(2), Eigen::Vector3d::UnitZ()));

            driver->orientation = (yaw * pitch * roll).inverse();

            driver->m_hapticData.angle1[0] = driver->m_BaseAngle[0];
            driver->m_simuData.angle1[0] = driver->m_hapticData.angle1[0];
            driver->m_hapticData.angle1[1] = driver->m_BaseAngle[1];
            driver->m_simuData.angle1[1] = driver->m_hapticData.angle1[1];
            driver->m_hapticData.angle1[2] = driver->m_BaseAngle[2];
            driver->m_simuData.angle1[2] = driver->m_hapticData.angle1[2];
            driver->m_hapticData.angle2[0] = driver->m_ToolAngle[0];
            driver->m_simuData.angle2[0] = driver->m_hapticData.angle2[0];
            driver->m_hapticData.angle2[1] = driver->m_ToolAngle[1];
            driver->m_simuData.angle2[1] = driver->m_hapticData.angle2[1];
            driver->m_hapticData.angle2[2] = driver->m_ToolAngle[2];
            driver->m_simuData.angle2[2] = driver->m_hapticData.angle2[2];


            driver->m_hapticData.transform[0] = driver->m_Orientation[0][0];
            driver->m_hapticData.transform[1] = driver->m_Orientation[1][0];
            driver->m_hapticData.transform[2] = driver->m_Orientation[2][0];
            driver->m_hapticData.transform[3] = 0.0;
            driver->m_simuData.transform[0] = driver->m_hapticData.transform[0];
            driver->m_simuData.transform[1] = driver->m_hapticData.transform[1];
            driver->m_simuData.transform[2] = driver->m_hapticData.transform[2];
            driver->m_simuData.transform[3] = driver->m_hapticData.transform[3];


            driver->m_hapticData.transform[4] = driver->m_Orientation[0][1];
            driver->m_hapticData.transform[5] = driver->m_Orientation[1][1];
            driver->m_hapticData.transform[6] = driver->m_Orientation[2][1];
            driver->m_hapticData.transform[7] = 0.0;
            driver->m_simuData.transform[4] = driver->m_hapticData.transform[4];
            driver->m_simuData.transform[5] = driver->m_hapticData.transform[5];
            driver->m_simuData.transform[6] = driver->m_hapticData.transform[6];
            driver->m_simuData.transform[7] = driver->m_hapticData.transform[7];


            driver->m_hapticData.transform[8] = driver->m_Orientation[0][2];
            driver->m_hapticData.transform[9] = driver->m_Orientation[1][2];
            driver->m_hapticData.transform[10] = driver->m_Orientation[2][2];
            driver->m_hapticData.transform[11] = 0.0;
            driver->m_simuData.transform[8] = driver->m_hapticData.transform[8];
            driver->m_simuData.transform[9] = driver->m_hapticData.transform[9];
            driver->m_simuData.transform[10] = driver->m_hapticData.transform[10];
            driver->m_simuData.transform[11] = driver->m_hapticData.transform[11];


            //driver->m_Pos[0] = -driver->m_Pos[0];
            driver->m_hapticData.transform[12] = driver->m_Pos[0];
            driver->m_hapticData.transform[13] = driver->m_Pos[1];
            driver->m_hapticData.transform[14] = driver->m_Pos[2];
            driver->m_hapticData.transform[15] = 1.0;
            driver->m_simuData.transform[12] = driver->m_hapticData.transform[12];
            driver->m_simuData.transform[13] = driver->m_hapticData.transform[13];
            driver->m_simuData.transform[14] = driver->m_hapticData.transform[14];
            driver->m_simuData.transform[15] = driver->m_hapticData.transform[15];

            driver->m_fButtonPress = hfdGetButton(0);
            driver->m_hapticData.buttonState = driver->m_fButtonPress;
            driver->m_simuData.buttonState = driver->m_fButtonPress;

            Vector3 currentForce;
            if (driver->m_forceFeedback)
            {
                Vector3 pos(driver->m_Pos[0] * 0.1, 
                            driver->m_Pos[1] * 0.1,
                            driver->m_Pos[2] * 0.1);
                Vector3 pos_in_world =
                    driver->d_positionBase.getValue() +
                    driver->d_orientationBase.getValue().rotate(pos * driver->d_scale.getValue());

                driver->m_forceFeedback->computeForce(pos_in_world[0], pos_in_world[1],
                                                      pos_in_world[2], 0, 0, 0, 0, currentForce[0],
                                                      currentForce[1], currentForce[2]);
                driver->m_isInContact = false;
                for (int i = 0; i < 3; i++)
                    if (currentForce[i] != 0.0)
                    {
                        driver->m_isInContact = true;
                        break;
                    }
            }
            else
            {
                Vector3 inputForceFeedback = driver->d_inputForceFeedback.getValue();
                double normValue = inputForceFeedback.norm();
                double maxInputForceFeedback = driver->d_maxInputForceFeedback.getValue();

                if (maxInputForceFeedback > 0.0)
                {
                    if (normValue > maxInputForceFeedback)
                    {
                        msg_warning(driver)
                            << "Force given to applied inputForceFeedback (norm = " << normValue
                            << ") exceeds the maxInputForceFeedback (" << maxInputForceFeedback
                            << ")";

                        inputForceFeedback[0] *= maxInputForceFeedback / normValue;
                        inputForceFeedback[1] *= maxInputForceFeedback / normValue;
                        inputForceFeedback[2] *= maxInputForceFeedback / normValue;

                        driver->d_inputForceFeedback.setValue(inputForceFeedback);
                    }
                    currentForce = driver->d_inputForceFeedback.getValue();
                }
                else
                {
                    msg_error(driver) << "maxInputForceFeedback value (" << maxInputForceFeedback
                                      << ") is negative or 0, it should be strictly positive";
                }
            }

            Vector3 force_in_omni =
                driver->d_orientationBase.getValue().inverseRotate(currentForce) *
                driver->d_forceScale.getValue();

            
            driver->omni_force[0] = force_in_omni[0];
            driver->omni_force[1] = force_in_omni[1];
            driver->omni_force[2] = force_in_omni[2];

            if (driver->m_bForceEnable && driver->m_bDeviceEnable)
            {
                hfdSetForce(driver->omni_force[0], driver->omni_force[1], driver->omni_force[2]);
                driver->feedbackflag = 13;
            }
        }
    }

    driver->m_bThreadRunning = false;
    //?
    return 0;
}

////#endif

//constructeur
HFDDriver::HFDDriver()
    : d_deviceName(initData(&d_deviceName, std::string("Default Device"), "deviceName","Name of device Configuration"))
    , d_positionBase(initData(&d_positionBase, Vec3(0,0,0), "positionBase","Position of the device base in the SOFA scene world coordinates"))
    , d_orientationBase(initData(&d_orientationBase, Quat(0,0,0,1), "orientationBase","Orientation of the device base in the SOFA scene world coordinates"))
    , d_orientationTool(initData(&d_orientationTool, Quat(0,0,0,1), "orientationTool","Orientation of the tool in the SOFA scene world coordinates"))
    , d_scale(initData(&d_scale, 1.0, "scale", "Default scale applied to the Device coordinates"))
    , d_forceScale(initData(&d_forceScale, 1.0, "forceScale", "Default forceScale applied to the force feedback. "))
    , d_maxInputForceFeedback(initData(&d_maxInputForceFeedback, double(1.0), "maxInputForceFeedback", "Maximum value of the normed input force feedback for device security"))
    , d_inputForceFeedback(initData(&d_inputForceFeedback, Vec3(0, 0, 0), "inputForceFeedback", "Input force feedback in case of no LCPForceFeedback is found (manual setting)"))
    , d_manualStart(initData(&d_manualStart, false, "manualStart", "If true, will not automatically initDevice at component init phase."))
    , d_emitButtonEvent(initData(&d_emitButtonEvent, false, "emitButtonEvent", "If true, will send event through the graph when button are pushed/released"))
    , d_frameVisu(initData(&d_frameVisu, false, "drawDeviceFrame", "Visualize the frame corresponding to the device tooltip"))
    , d_omniVisu(initData(&d_omniVisu, false, "drawDevice", "Visualize the HFD device in the virtual scene"))    
    , d_posDevice(initData(&d_posDevice, "positionDevice", "position of the base of the part of the device"))
    , d_angle(initData(&d_angle, "angle", "Angluar values of joint (rad)"))
    , d_button_1(initData(&d_button_1,"button1","Button state 1"))
    , d_button_2(initData(&d_button_2,"button2","Button state 2"))    
    , l_forceFeedback(initLink("forceFeedBack", "link to the forceFeedBack component, if not set will search through graph and take first one encountered."))
    , m_simulationStarted(false)
    , m_isInContact(false)
    , m_hHD(0)
{
    this->f_listening.setValue(true);
    m_forceFeedback = nullptr;
    m_HFDVisualModel = std::make_unique<HFDVisualModel>();
    sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Loading);

        //! some flags indicate device state.
    m_bOpen = false;
    m_bInit = false;
    m_bCalibrate = false;
    m_bForceEnable = false;
    m_bDeviceEnable = false;
    m_bThreadFinish = true;
    m_bThreadRunning = false;
    for (int i = 0; i < 3; i++)
    {
        m_BaseAngle[i] = 0;
        m_ToolAngle[i] = 0;
        m_Pos[i] = 0;
        m_Force[i] = 0;
        for (int j = 0; j < 3; j++)
        {
            m_Orientation[i][j] = 0;
        }
    }
    m_fButtonPress = false;
}


HFDDriver::~HFDDriver()
{
    clearDevice();
}


//executed once at the start of Sofa, initialization of all variables excepts haptics-related ones
void HFDDriver::init()
{
    // 1- retrieve ForceFeedback component pointer
    if (l_forceFeedback.empty())
    {
        simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext()); // access to current node
        m_forceFeedback = context->get<sofa::component::haptics::ForceFeedback>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);
    }
    else
    {
        m_forceFeedback = l_forceFeedback.get();
    }

    if (!m_forceFeedback.get())
    {
        msg_warning() << "No forceFeedBack component found in the scene. Only the motion of the haptic tool will be simulated.";
    }


    // 2- init device and Hd scheduler
    if (d_manualStart.getValue() == false)
        initDevice();
}


void HFDDriver::clearDevice()
{
    m_bThreadFinish = true;
    m_bThreadFinish = true;
    int i = 200;
    while (i--)
        ;
    hfdClose();
    m_bThreadFinish = true;
    m_bThreadFinish = true;
    m_bThreadFinish = true;
    int j = 500;
    while (j--)
        ;
    hfdClose();
    m_bThreadFinish = true;
    m_bThreadFinish = true;
    m_bThreadFinish = true;
    int k = 1000;
    while (k--)
        ;
    m_bThreadFinish = true;
    m_bThreadFinish = true;
    hfdClose();
    for (int m = 0; m < 100; m++)
    {
        m_bThreadFinish = true;
        hfdClose();
    }
}


void HFDDriver::initDevice()
{
    if (m_bThreadRunning)
    {
        m_bThreadFinish = true;
    }

    int error = 0;

    //! open the hfd haptic device.
    error = hfdOpen();
    if (error < 0)
    {
        msg_warning() << "hfd Device Open Failed!";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(
            sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }

    m_bOpen = true;

    //! initialize the hfd haptic device.
    error = hfdInit();
    if (error == 0)
    {
        msg_warning() << "hfd Device Init Failed!";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(
            sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }
    m_bInit = true;

        error = hfdCalibrateDevice();
    if (error == 0)
    {
        msg_warning() << "Calibrate the hfd Haptic Device Failed!";
        return ;
    }
    m_bCalibrate = true;

    error = hfdEnableForce(HFD_ON);
    if (error == 0)
    {
        msg_warning() << "Failed to Enable Force Mode!";
        return ;
    }
    m_bForceEnable = true;

    hfdEnableExpertMode();

    error = hfdEnableDevice(true);
    if (error == 0)
    {
        msg_warning() << "Failed to Enable Haptice Device!";
        return ;
    }
    m_bDeviceEnable = true;

    error = hfdSetGravityCompensation(HFD_ON);
    if (error == 0)
    {
        msg_warning() << "Failed to Set Gravity Compensation!";
        return ;
    }

    error = hfdStartThread(stateCallback, this, 1);
    if (error == 0)
    {
        msg_warning() << "Refresh thread of hfd device faile to create!";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(
            sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }

    updatePosition();
}


void HFDDriver::updatePosition()
{
    type::Vector6 & angle = *d_angle.beginEdit();
    HFDDriver::Coord & posDevice = *d_posDevice.beginEdit();

    const Vec3 & positionBase = d_positionBase.getValue();
    const Quat & orientationBase = d_orientationBase.getValue();
    const Quat & orientationTool = d_orientationTool.getValue();
    const double & scale = d_scale.getValue();

    // update button state
    updateButtonStates();

    //copy angle
    angle[0] = m_simuData.angle1[0];
    angle[1] = m_simuData.angle1[1];
    angle[2] = -(M_PI/2)+m_simuData.angle1[2]-m_simuData.angle1[1];
    angle[3] = -(M_PI/2)-m_simuData.angle2[0];
    angle[4] = m_simuData.angle2[1];
    angle[5] = -(M_PI/2)-m_simuData.angle2[2];

    Vec3 position;
    position[0] = m_simuData.transform[12+0] * 0.1;
    position[1] = m_simuData.transform[12+1] * 0.1;
    position[2] = m_simuData.transform[12+2] * 0.1;

    Quat Orientation;
    Orientation[0] = orientation.x();
    Orientation[1] = orientation.y();
    Orientation[2] = orientation.z();
    Orientation[3] = orientation.w();
    //orientation.fromMatrix(mrot);

    //compute the position of the tool (according to positionbase, orientation base and the scale
    posDevice.getCenter() = positionBase + orientationBase.rotate(position*scale);
    posDevice.getOrientation() = orientationBase * Orientation * orientationTool;

    d_posDevice.endEdit();
    d_angle.endEdit();


    if (d_omniVisu.getValue() && m_HFDVisualModel != nullptr)
    {
        if (!m_HFDVisualModel->isDisplayInitiate()) // first time, need to init visualModel first
        {
            sofa::simulation::Node::SPtr rootContext = static_cast<simulation::Node*>(this->getContext()->getRootContext());
            m_HFDVisualModel->initDisplay(rootContext, d_deviceName.getValue(), d_scale.getValue());            
        }

        if (!m_HFDVisualModel->isDisplayActivated())
            m_HFDVisualModel->activateDisplay(true);

        m_HFDVisualModel->updateDisplay(d_posDevice.getValue(), m_hapticData.angle1, m_hapticData.angle2);
    }
    else if (d_omniVisu.getValue() == false && m_HFDVisualModel && m_HFDVisualModel->isDisplayActivated())
    {
        m_HFDVisualModel->activateDisplay(false);
    }
}


void HFDDriver::updateButtonStates()
{
    int nbrButton = 2;
    sofa::type::fixed_array<bool, 2> buttons;
    buttons[0] = d_button_1.getValue();
    buttons[1] = d_button_2.getValue();

    //copy button state
    sofa::type::fixed_array<bool, 2> oldStates;
    for (int i = 0; i < nbrButton; i++)
        oldStates[i] = buttons[i];

    // get new values
    ////#if HFD_HAVE_OPENHAPTICS
    buttons[0] = m_simuData.buttonState;
    buttons[1] = m_simuData.buttonState;
    //buttons[0] = m_simuData.buttonState & HD_DEVICE_BUTTON_1;
    //buttons[1] = m_simuData.buttonState & HD_DEVICE_BUTTON_2;
    ////#endif

    d_button_1.setValue(buttons[0]);
    d_button_2.setValue(buttons[1]);

    // emit event if requested
    if (!d_emitButtonEvent.getValue())
        return;

    sofa::simulation::Node::SPtr rootContext = static_cast<simulation::Node*>(this->getContext()->getRootContext());
    if (!rootContext)
    {
        msg_error() << "Rootcontext can't be found using this->getContext()->getRootContext()";
        return;
    }

    for (int i = 0; i < nbrButton; i++)
    {
        std::string eventString;
        if (buttons[i] && !oldStates[i]) // button pressed
            eventString = "button" + std::to_string(i) + "pressed";
        else if (!buttons[i] && oldStates[i]) // button released
            eventString = "button" + std::to_string(i) + "released";

        if (!eventString.empty())
        {
            sofa::core::objectmodel::ScriptEvent eventS(static_cast<simulation::Node*>(this->getContext()), eventString.c_str());
            rootContext->propagateEvent(core::ExecParams::defaultInstance(), &eventS);
        }
    }  
}


void HFDDriver::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (sofa::core::objectmodel::BaseObject::d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return;

    vparams->drawTool()->saveLastState();

    if (d_frameVisu.getValue())
    {
        vparams->drawTool()->disableLighting();

        const HFDDriver::Coord& posDevice = d_posDevice.getValue();

        float glRadius = (float)d_scale.getValue()*0.1f;
        vparams->drawTool()->drawArrow(posDevice.getCenter(), posDevice.getCenter() + posDevice.getOrientation().rotate(type::Vector3(2,0,0)*d_scale.getValue()), glRadius, sofa::type::RGBAColor::red() );
        vparams->drawTool()->drawArrow(posDevice.getCenter(), posDevice.getCenter() + posDevice.getOrientation().rotate(type::Vector3(0,2,0)*d_scale.getValue()), glRadius, sofa::type::RGBAColor::green() );
        vparams->drawTool()->drawArrow(posDevice.getCenter(), posDevice.getCenter() + posDevice.getOrientation().rotate(type::Vector3(0,0,2)*d_scale.getValue()), glRadius, sofa::type::RGBAColor::blue() );
    }

    if (d_omniVisu.getValue() && m_HFDVisualModel != nullptr)
        m_HFDVisualModel->drawDevice(d_button_1.getValue(), d_button_2.getValue());

    vparams->drawTool()->restoreLastState();
}


void HFDDriver::computeBBox(const core::ExecParams*  params, bool  )
{
    SReal minBBox[3] = {1e10,1e10,1e10};
    SReal maxBBox[3] = {-1e10,-1e10,-1e10};

    minBBox[0] = d_posDevice.getValue().getCenter()[0]-d_positionBase.getValue()[0]*d_scale.getValue();
    minBBox[1] = d_posDevice.getValue().getCenter()[1]-d_positionBase.getValue()[1]*d_scale.getValue();
    minBBox[2] = d_posDevice.getValue().getCenter()[2]-d_positionBase.getValue()[2]*d_scale.getValue();

    maxBBox[0] = d_posDevice.getValue().getCenter()[0]+d_positionBase.getValue()[0]*d_scale.getValue();
    maxBBox[1] = d_posDevice.getValue().getCenter()[1]+d_positionBase.getValue()[1]*d_scale.getValue();
    maxBBox[2] = d_posDevice.getValue().getCenter()[2]+d_positionBase.getValue()[2]*d_scale.getValue();

    this->f_bbox.setValue(sofa::type::TBoundingBox<SReal>(minBBox,maxBBox));
}


void HFDDriver::handleEvent(core::objectmodel::Event *event)
{
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
        //if (sofa::core::objectmodel::BaseObject::d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        //    return;

        //if (m_hStateHandles.size() && m_hStateHandles[0] == 0xFFFFFFFF)
        //    return;

        m_simulationStarted = true;
        updatePosition();
    }
}


int HFDDriverClass = core::RegisterObject("Driver allowing interfacing with HFD haptic devices.")
.add< HFDDriver >()
.addAlias("DefaultHapticsDevice")
;

} // namespace sofa::component::controller
