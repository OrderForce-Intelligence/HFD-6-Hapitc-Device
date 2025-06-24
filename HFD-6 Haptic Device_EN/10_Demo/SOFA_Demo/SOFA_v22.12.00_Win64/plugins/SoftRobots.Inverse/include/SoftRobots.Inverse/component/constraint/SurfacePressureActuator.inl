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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_SURFACEPRESSUREACTUATOR_INL
#define SOFA_COMPONENT_CONSTRAINTSET_SURFACEPRESSUREACTUATOR_INL

#include "SurfacePressureActuator.h"

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
SurfacePressureActuator<DataTypes>::SurfacePressureActuator(MechanicalState* object)
    : Actuator<DataTypes>(object)
    , SurfacePressureModel<DataTypes>(object)
    , d_initPressure(initData(&d_initPressure,Real(0.0), "initPressure",
                          "Initial pressure if any. Default is 0."))
{
    // These datas from SurfacePressureModel have no sense for actuator
    d_eqPressure.setDisplayed(false);
    d_eqVolumeGrowth.setDisplayed(false);

    // QP on only one value, we set dimension to one
    m_lambdaInit.resize(1);
    m_deltaMax.resize(1);
    m_deltaMin.resize(1);
    m_lambdaMax.resize(1);
    m_lambdaMin.resize(1);
}

template<class DataTypes>
SurfacePressureActuator<DataTypes>::~SurfacePressureActuator()
{
}

template<class DataTypes>
void SurfacePressureActuator<DataTypes>::init()
{
    SurfacePressureModel<DataTypes>::init();
    initDatas();
    initLimits();
}

template<class DataTypes>
void SurfacePressureActuator<DataTypes>::reinit()
{
    SurfacePressureModel<DataTypes>::reinit();
    initDatas();
    initLimits();
}

template<class DataTypes>
void SurfacePressureActuator<DataTypes>::reset()
{
    SurfacePressureModel<DataTypes>::reset();
    initDatas();
    initLimits();
}

template<class DataTypes>
void SurfacePressureActuator<DataTypes>::initDatas()
{
    d_volumeGrowth.setValue(0.0);
    d_pressure.setValue(d_initPressure.getValue());

    if(d_initPressure.isSet()){
        m_hasLambdaInit = true;
        m_lambdaInit[0] = d_initPressure.getValue();
    }
}

template<class DataTypes>
void SurfacePressureActuator<DataTypes>::initLimits()
{
    ReadAccessor<Data<Real>> maxVolumeGrowthVariation = d_maxVolumeGrowthVariation;
    ReadAccessor<Data<Real>> maxVolumeGrowth = d_maxVolumeGrowth;
    ReadAccessor<Data<Real>> minVolumeGrowth = d_minVolumeGrowth;
    ReadAccessor<Data<Real>> maxPressure = d_maxPressure;
    ReadAccessor<Data<Real>> minPressure = d_minPressure;

    if(d_maxPressure.isSet())
    {
        m_hasLambdaMax = true;
        m_lambdaMax[0] = maxPressure;
    }

    if(d_minPressure.isSet())
    {
        m_hasLambdaMin = true;
        m_lambdaMin[0] = minPressure;
    }

    if(!m_hasLambdaMin || m_lambdaMin[0]<0)
        msg_info() << "A negative pressure will empty/drain the cavity. If you do not want this feature set 'minPressure=0'.";

    if(d_maxVolumeGrowth.isSet())
    {
        m_hasDeltaMax = true;
        m_deltaMax[0] = maxVolumeGrowth;
    }

    if(d_minVolumeGrowth.isSet())
    {
        m_hasDeltaMin = true;
        m_deltaMin[0] = minVolumeGrowth;
    }

    if (d_maxVolumeGrowthVariation.isSet())
    {
        m_hasDeltaMax = true;
        m_hasDeltaMin = true;
        if (m_deltaMax[0] >= maxVolumeGrowthVariation || !d_maxVolumeGrowth.isSet())
            m_deltaMax[0] = maxVolumeGrowthVariation;
        if (m_deltaMin[0] <= -maxVolumeGrowthVariation || !d_minVolumeGrowth.isSet())
            m_deltaMin[0] = -maxVolumeGrowthVariation;
    }
}


template<class DataTypes>
void SurfacePressureActuator<DataTypes>::updateLimits()
{
    ReadAccessor<Data<double>> volumeGrowth = d_volumeGrowth;
    ReadAccessor<Data<Real>> maxVolumeGrowthVariation = d_maxVolumeGrowthVariation;
    ReadAccessor<Data<Real>> maxVolumeGrowth = d_maxVolumeGrowth;
    ReadAccessor<Data<Real>> minVolumeGrowth = d_minVolumeGrowth;

    if(d_maxVolumeGrowth.isSet())
        m_deltaMax[0] = maxVolumeGrowth;

    if(d_minVolumeGrowth.isSet())
        m_deltaMin[0] = minVolumeGrowth;

    if(d_maxVolumeGrowthVariation.isSet())
    {
        if(rabs(m_deltaMax[0] - volumeGrowth) >= maxVolumeGrowthVariation || !d_maxVolumeGrowth.isSet())
            m_deltaMax[0] = volumeGrowth + maxVolumeGrowthVariation;
        if(rabs(m_deltaMin[0] + volumeGrowth) <= -maxVolumeGrowthVariation || !d_minVolumeGrowth.isSet())
            m_deltaMin[0] = volumeGrowth - maxVolumeGrowthVariation;
    }
}

template<class DataTypes>
void SurfacePressureActuator<DataTypes>::storeResults(type::vector<double> &lambda,
                                                      type::vector<double> &delta)
{
    d_pressure.setValue(lambda[0]);
    d_volumeGrowth.setValue(delta[0]);

    updateLimits();

    Actuator<DataTypes>::storeResults(lambda, delta);
}

template<class DataTypes>
void SurfacePressureActuator<DataTypes>::storeLambda(const ConstraintParams* cParams,
                                                       core::MultiVecDerivId res,
                                                       const BaseVector* lambda)
{
    SOFA_UNUSED(cParams);
    SOFA_UNUSED(res);
    SOFA_UNUSED(lambda);

    // Do nothing, because the position of the mechanical state is not up to date when storeLambda() is called.
    // So if we compute delta from SurfacePressureModel::storeLambda() we would be one step behind.
    // Instead the actual delta is stored in storeResults(), which is called from QPInverseProblemSolver
}


} // namespace constraintset

} // namespace component

} // namespace sofa

#endif
