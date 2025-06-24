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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_SURFACEPRESSUREEQUALITY_INL
#define SOFA_COMPONENT_CONSTRAINTSET_SURFACEPRESSUREEQUALITY_INL

#include "SurfacePressureEquality.h"

#include <sofa/helper/logging/Messaging.h>

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::type::Vec3d;
using type::vector;
using sofa::helper::rabs;

template<class DataTypes>
SurfacePressureEquality<DataTypes>::SurfacePressureEquality(MechanicalState* object)
    : Equality<DataTypes>(object)
    , SurfacePressureModel<DataTypes>(object)
{
    d_maxVolumeGrowthVariation.setDisplayed(false);
    d_maxVolumeGrowth.setDisplayed(false);
    d_minVolumeGrowth.setDisplayed(false);
    d_maxPressure.setDisplayed(false);
    d_minPressure.setDisplayed(false);

    // QP on only one value, we set dimension to one
    m_lambdaEqual.resize(1);
    m_deltaEqual.resize(1);
}

template<class DataTypes>
SurfacePressureEquality<DataTypes>::~SurfacePressureEquality()
{
}

template<class DataTypes>
void SurfacePressureEquality<DataTypes>::init()
{
    SurfacePressureModel<DataTypes>::init();
    updateConstraint();
}

template<class DataTypes>
void SurfacePressureEquality<DataTypes>::reinit()
{
    updateConstraint();
}

template<class DataTypes>
void SurfacePressureEquality<DataTypes>::reset()
{
    SurfacePressureModel<DataTypes>::reset();
    d_volumeGrowth.setValue(0.0);
    d_pressure.setValue(0.0);

    updateConstraint();
}

template<class DataTypes>
void SurfacePressureEquality<DataTypes>::updateConstraint()
{
    ReadAccessor<Data<Real>> eqPressure = d_eqPressure;
    ReadAccessor<Data<Real>> eqVolumeGrowth = d_eqVolumeGrowth;

    if(d_eqPressure.isSet())
    {
        m_hasLambdaEqual= true;
        m_lambdaEqual[0] = eqPressure;
    }

    if(d_eqVolumeGrowth.isSet())
    {
        m_hasDeltaEqual = true;
        m_deltaEqual[0] = eqVolumeGrowth;
    }
}

template<class DataTypes>
void SurfacePressureEquality<DataTypes>::storeLambda(const ConstraintParams* cParams,
                                                     core::MultiVecDerivId res,
                                                     const BaseVector* lambda)
{
    SurfacePressureModel<DataTypes>::storeLambda(cParams,res,lambda);
    updateConstraint();
}


} // namespace constraintset

} // namespace component

} // namespace sofa

#endif
