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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_CABLEEFFECTOR_INL
#define SOFA_COMPONENT_CONSTRAINTSET_CABLEEFFECTOR_INL

#include <sofa/core/visual/VisualParams.h>

#include "CableEffector.h"

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::core::visual::VisualParams;
using sofa::linearalgebra::BaseVector;

template<class DataTypes>
CableEffector<DataTypes>::CableEffector(MechanicalState* object)
    : Effector<DataTypes>(object)
    , CableModel<DataTypes>(object)
    , d_desiredLength(initData(&d_desiredLength, "desiredLength", ""))
{
    setUpData();
}


template<class DataTypes>
CableEffector<DataTypes>::~CableEffector()
{
}

template<class DataTypes>
void CableEffector<DataTypes>::setUpData()
{
    // These datas from CableModel have no sense for effector
    d_maxForce.setDisplayed(false);
    d_minForce.setDisplayed(false);
    d_maxDispVariation.setDisplayed(false);
    d_maxPositiveDisplacement.setDisplayed(false);
    d_maxNegativeDisplacement.setDisplayed(false);
    d_eqDisplacement.setDisplayed(false);
    d_eqForce.setDisplayed(false);
}

template<class DataTypes>
void CableEffector<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                      BaseVector *resV,
                                                      const BaseVector *Jdx)
{
    SOFA_UNUSED(cParams);

    d_cableLength.setValue(getCableLength(m_state->readPositions().ref()));
    Real desiredLength = getTarget(d_desiredLength.getValue(), d_cableLength.getValue());
    Real dfree = Jdx->element(0) + desiredLength - d_cableLength.getValue();
    resV->set(m_constraintId, dfree);
}

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif
