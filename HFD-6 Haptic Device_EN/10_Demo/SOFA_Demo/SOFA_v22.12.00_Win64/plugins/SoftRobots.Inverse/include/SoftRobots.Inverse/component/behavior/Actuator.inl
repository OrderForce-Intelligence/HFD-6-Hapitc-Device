/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                              SOFA :: Framework                              *
*                                                                             *
* Authors: The SOFA Team (see Authors.txt)                                    *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_CORE_BEHAVIOR_ACTUATOR_INL
#define SOFA_CORE_BEHAVIOR_ACTUATOR_INL

#include <SoftRobots.Inverse/component/behavior/Actuator.h>

namespace sofa
{

namespace core
{

namespace behavior
{

template<class DataTypes>
Actuator<DataTypes>::Actuator(MechanicalState<DataTypes> *mm)
    : SoftRobotsConstraint<DataTypes>(mm),
      d_applyForce(initData(&d_applyForce, true, "applyForce", "If false, no force will be applied. Default value is true."))
{
    m_constraintType = ACTUATOR;
}

template<class DataTypes>
Actuator<DataTypes>::~Actuator()
{
}

template<class DataTypes>
void Actuator<DataTypes>::storeResults(type::vector<double> &lambda, type::vector<double> &delta)
{
    SOFA_UNUSED(delta);

    if (!d_applyForce.getValue())
        for (unsigned int i=0; i<m_nbLines; i++)
            lambda[i] = 0.;
}


} // namespace behavior

} // namespace core

} // namespace sofa

#endif // SOFA_CORE_BEHAVIOR_ACTUATOR_INL
