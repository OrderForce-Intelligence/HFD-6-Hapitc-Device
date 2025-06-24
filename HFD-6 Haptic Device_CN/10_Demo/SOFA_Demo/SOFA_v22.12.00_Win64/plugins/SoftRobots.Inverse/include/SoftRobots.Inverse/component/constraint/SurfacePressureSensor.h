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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_SURFACEPRESSURESENSOR_H
#define SOFA_COMPONENT_CONSTRAINTSET_SURFACEPRESSURESENSOR_H

#include <SoftRobots.Inverse/component/behavior/Sensor.h>
#include <SoftRobots/component/constraint/model/SurfacePressureModel.h>
#include <sofa/core/behavior/ConstraintResolution.h>

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::core::behavior::Sensor;
using sofa::helper::ReadAccessor;
using sofa::core::VecCoordId;
using sofa::core::ConstraintParams;

using sofa::core::behavior::ConstraintResolution ;

/**
 * This component allows the user to get the (change of) volume of a cavity
 * TODO: In the future we should also aim for giving the (change of) pressure, which
 * TODO -> Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/

class DoNothingConstraintResolution : public ConstraintResolution
{
public:
    DoNothingConstraintResolution();

//////////////////// Inherited from ConstraintResolution ////////////////////
void init(int line, double** w, double *lambda);
void resolution(int line, double** w, double* d, double* lambda, double* dfree);
/////////////////////////////////////////////////////////////////////////////

};

template< class DataTypes >
class SurfacePressureSensor : public Sensor<DataTypes> , public SurfacePressureModel<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SurfacePressureSensor,DataTypes), SOFA_TEMPLATE(Sensor,DataTypes));

    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;

public:    
    SurfacePressureSensor(MechanicalState* object = nullptr);
    ~SurfacePressureSensor() override;

    /////////////////// from sofa::core::behavior::Constraint /////////////////////////
    void getConstraintResolution(std::vector<ConstraintResolution*>& resTab,
                                 unsigned int& offset) override;

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.

    using SurfacePressureModel<DataTypes>::d_eqPressure;
    using SurfacePressureModel<DataTypes>::d_eqVolumeGrowth;
    using SurfacePressureModel<DataTypes>::d_minPressure;
    using SurfacePressureModel<DataTypes>::d_maxPressure;
    using SurfacePressureModel<DataTypes>::d_maxPressureVariation;
    using SurfacePressureModel<DataTypes>::d_minVolumeGrowth;
    using SurfacePressureModel<DataTypes>::d_maxVolumeGrowth;
    using SurfacePressureModel<DataTypes>::d_maxVolumeGrowthVariation;
    ////////////////////////////////////////////////////////////////////////////

protected:

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.    
    ////////////////////////////////////////////////////////////////////////////

};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#if !defined(SOFTROBOTS_INVERSE_SURFACEPRESSURESENSOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API SurfacePressureSensor<sofa::defaulttype::Vec3Types>;
#endif

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_SURFACEPRESSURESENSOR_H
