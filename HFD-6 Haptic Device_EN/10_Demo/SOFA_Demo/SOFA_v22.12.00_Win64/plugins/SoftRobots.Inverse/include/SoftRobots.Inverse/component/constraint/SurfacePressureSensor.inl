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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_SURFACEPRESSURESENSOR_INL
#define SOFA_COMPONENT_CONSTRAINTSET_SURFACEPRESSURESENSOR_INL

#include <sofa/core/visual/VisualParams.h>

#include "SurfacePressureSensor.h"

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::core::visual::VisualParams;
using sofa::linearalgebra::BaseVector;
using type::vector;


DoNothingConstraintResolution::DoNothingConstraintResolution()
    : ConstraintResolution(1)
{
}


void DoNothingConstraintResolution::init(int line, double** w, double *lambda)
{
    SOFA_UNUSED(line);
    SOFA_UNUSED(w);
    SOFA_UNUSED(lambda);
}
void DoNothingConstraintResolution::resolution(int line, double** w, double* d, double* lambda, double* dfree)
{
    SOFA_UNUSED(line);
    SOFA_UNUSED(w);
    SOFA_UNUSED(d);
    SOFA_UNUSED(lambda);
    SOFA_UNUSED(dfree);
}


template<class DataTypes>
SurfacePressureSensor<DataTypes>::SurfacePressureSensor(MechanicalState* object)
    : Sensor<DataTypes>(object)
    , SurfacePressureModel<DataTypes>(object)
{
    // These datas from SurfacePressureModel have no sense for sensor
    d_eqPressure.setDisplayed(false);
    d_eqVolumeGrowth.setDisplayed(false);
    d_minPressure.setDisplayed(false);
    d_maxPressure.setDisplayed(false);
    d_maxPressureVariation.setDisplayed(false);
    d_minVolumeGrowth.setDisplayed(false);
    d_maxVolumeGrowth.setDisplayed(false);
    d_maxVolumeGrowthVariation.setDisplayed(false);
}

template<class DataTypes>
SurfacePressureSensor<DataTypes>::~SurfacePressureSensor()
{
}

template<class DataTypes>
void SurfacePressureSensor<DataTypes>::getConstraintResolution(std::vector<ConstraintResolution*>& resTab,unsigned int& offset)
{
    DoNothingConstraintResolution *cr=  new DoNothingConstraintResolution();
    resTab[offset++] = cr;
}


} // namespace constraintset

} // namespace component

} // namespace sofa

#endif
