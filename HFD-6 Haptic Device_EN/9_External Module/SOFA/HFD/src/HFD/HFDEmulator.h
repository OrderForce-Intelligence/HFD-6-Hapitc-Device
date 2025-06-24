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
#pragma once

#include <HFD/config.h>
#include <HFD/HFDDriver.h>

#include <sofa/simulation/TaskScheduler.h>
#include <sofa/simulation/InitTasks.h>


namespace sofa::component::controller
{

using namespace sofa::defaulttype;
using namespace sofa::simulation;
using core::objectmodel::Data;


class HFDEmulator;

class SOFA_HFD_API HFDEmulatorTask : public CpuTask
{
public:
    HFDEmulatorTask(HFDEmulator* ptr, CpuTask::Status* pStatus);

    virtual ~HFDEmulatorTask() {}

    virtual MemoryAlloc run() override final;

private:
    HFDEmulator * m_driver;
};


/**
* HFD emulator class
*/
class SOFA_HFD_API HFDEmulator : public HFDDriver
{

public:
    SOFA_CLASS(HFDEmulator, HFDDriver);
    typedef RigidTypes::Coord Coord;
    typedef RigidTypes::VecCoord VecCoord;

    HFDEmulator();

    /// Public method to init tool. Can be called from thirdparty if @sa d_manualStart is set to true
    virtual void initDevice() override;

    /// Method to clear sheduler and free device. Called by default at driver destruction
    virtual void clearDevice() override;



    Data <SReal> d_speedFactor; ///< factor to increase/decrease the movements speed    

    void applyTranslation(sofa::type::Vec3 translation);
    void worldToLocal(sofa::type::Vec3& vector);

    void moveJoint1(SReal value);
    void moveJoint2(SReal value);
    void moveJoint3(SReal value);
    void moveGimbalAngle1(SReal value);
    void moveGimbalAngle2(SReal value);
    void moveGimbalAngle3(SReal value);

    void computeTransform();

    sofa::type::fixed_array<bool, 2> oldStates;

    
    void handleEvent(core::objectmodel::Event *) override;    

public:
    sofa::simulation::TaskScheduler* _taskScheduler;
    sofa::simulation::CpuTask::Status _simStepStatus;
    sofa::type::Vec3 m_toolForceFeedBack;

    std::mutex lockPosition;

    bool m_terminate;

    type::Vec3 m_toolPosition;
};

} // sofa::component::controller
