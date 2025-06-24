/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
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
*                               SOFA :: Modules                               *
*                                                                             *
* This component is not open-source                                           *
*                                                                             *
* Authors: Christian Duriez                                                   *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_CONSTRAINTSET_CABLESENSOR_INL
#define SOFA_COMPONENT_CONSTRAINTSET_CABLESENSOR_INL

#include <sofa/core/visual/VisualParams.h>

#include "CableSensor.h"

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::core::visual::VisualParams;
using sofa::linearalgebra::BaseVector;
using type::vector;

template<class DataTypes>
CableSensor<DataTypes>::CableSensor(MechanicalState* object)
    : Sensor<DataTypes>(object)
    , CableModel<DataTypes>(object)
{
    setUpData();
}


template<class DataTypes>
CableSensor<DataTypes>::~CableSensor()
{
}


template<class DataTypes>
void CableSensor<DataTypes>::setUpData()
{
    // These datas from CableModel have no sense for sensor
    d_maxForce.setDisplayed(false);
    d_minForce.setDisplayed(false);
    d_eqForce.setDisplayed(false);
    d_force.setDisplayed(false);
    d_maxPositiveDisplacement.setDisplayed(false);
    d_maxNegativeDisplacement.setDisplayed(false);
    d_maxDispVariation.setDisplayed(false);
    d_eqDisplacement.setDisplayed(false);
    d_displacement.setDisplayed(false);
}


} // namespace constraintset

} // namespace component

} // namespace sofa

#endif
