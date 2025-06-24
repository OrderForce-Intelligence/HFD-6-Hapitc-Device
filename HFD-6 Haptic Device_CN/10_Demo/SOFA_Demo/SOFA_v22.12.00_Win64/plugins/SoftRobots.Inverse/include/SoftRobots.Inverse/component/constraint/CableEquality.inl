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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_CABLEEQUALITY_INL
#define SOFA_COMPONENT_CONSTRAINTSET_CABLEEQUALITY_INL

#include "CableEquality.h"

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::linearalgebra::BaseVector;
using sofa::helper::ReadAccessor;
using sofa::helper::rabs;

template<class DataTypes>
CableEquality<DataTypes>::CableEquality(MechanicalState* object)
    : Equality<DataTypes>(object)
    , CableModel<DataTypes>(object)
    , d_constrainAtTime(initData(&d_constrainAtTime,Real(0.0), "constrainAtTime",
                          "No constraints will be applied before this time. \n"
                          "Example of use: to allow the cable to reach an \n"
                          "initial configuration before optimizing."))

    , d_displayCableLimit(initData(&d_displayCableLimit,false, "displayCableLimit",
                          "Display cable in red when the limit set by user has been reach."))
{
    d_displayCableLimit.setGroup("Visualization");
    d_maxForce.setDisplayed(false);
    d_minForce.setDisplayed(false);
    d_maxPositiveDisplacement.setDisplayed(false);
    d_maxNegativeDisplacement.setDisplayed(false);
    d_maxDispVariation.setDisplayed(false);

    // QP on only one value, we set dimension to one
    m_lambdaEqual.resize(1);
    m_deltaEqual.resize(1);
}


template<class DataTypes>
CableEquality<DataTypes>::~CableEquality()
{
}

template<class DataTypes>
void CableEquality<DataTypes>::init()
{
    CableModel<DataTypes>::init();
    updateConstraint();
}


template<class DataTypes>
void CableEquality<DataTypes>::reinit()
{
    CableModel<DataTypes>::reinit();
    updateConstraint();
}

template<class DataTypes>
void CableEquality<DataTypes>::reset()
{
    d_displacement.setValue(0.0);
    d_force.setValue(0.0);
    updateConstraint();
}

template<class DataTypes>
void CableEquality<DataTypes>::updateConstraint()
{
    Real time = this->getContext()->getTime();
    if(time < d_constrainAtTime.getValue())
        return;

    ReadAccessor<Data<Real>> eqDisplacement = d_eqDisplacement;
    ReadAccessor<Data<Real>> eqForce = d_eqForce;


    if (d_eqDisplacement.isSet())
    {
        m_hasDeltaEqual = true;
        m_deltaEqual[0] = eqDisplacement;
    }

    if(d_eqForce.isSet())
    {
        m_hasLambdaEqual = true;
        m_lambdaEqual[0] = eqForce;
    }
}

template<class DataTypes>
void CableEquality<DataTypes>::storeLambda(const ConstraintParams* cParams,
                                           core::MultiVecDerivId res,
                                           const BaseVector* lambda)
{
    CableModel<DataTypes>::storeLambda(cParams,res,lambda);
    updateConstraint();
}


} // namespace constraintset

} // namespace component

} // namespace sofa

#endif
