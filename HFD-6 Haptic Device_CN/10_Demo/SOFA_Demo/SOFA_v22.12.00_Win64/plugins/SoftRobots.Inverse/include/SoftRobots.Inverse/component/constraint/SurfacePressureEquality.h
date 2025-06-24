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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_SURFACEPRESSUREEQUALITY_H
#define SOFA_COMPONENT_CONSTRAINTSET_SURFACEPRESSUREEQUALITY_H

#include <SoftRobots/component/constraint/model/SurfacePressureModel.h>
#include <SoftRobots.Inverse/component/behavior/Equality.h>

#include <SoftRobots.Inverse/component/config.h>

namespace sofa
{

namespace component
{

namespace constraintset
{
    using sofa::core::behavior::Equality;
    using sofa::core::ConstraintParams;
    using sofa::linearalgebra::BaseVector;
    using sofa::core::ConstVecCoordId;

/**
 * This component is used to solve an effector constraint by applying pressure on surfaces (for exemple cavities).
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/
template< class DataTypes >
class SurfacePressureEquality : public Equality<DataTypes> , public SurfacePressureModel<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SurfacePressureEquality,DataTypes), SOFA_TEMPLATE(Equality,DataTypes));

    typedef typename DataTypes::Coord                       Coord;
    typedef typename Coord::value_type                      Real;
    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;

public:
    SurfacePressureEquality(MechanicalState* object = nullptr);
    ~SurfacePressureEquality() override;

    /////////////// Inherited from BaseObject ////////////////////
    void init() override;
    void reinit() override;
    void reset() override;
    /////////////////////////////////////////////////////////////

    ////////////////////////// Inherited from BaseConstraint ////////////////
    void storeLambda(const ConstraintParams* cParams,
                     core::MultiVecDerivId res,
                     const BaseVector* lambda) override;
    /////////////////////////////////////////////////////////////////////////

protected:

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using Equality<DataTypes>::m_state ;
    using SurfacePressureModel<DataTypes>::d_cavityVolume ;
    using SurfacePressureModel<DataTypes>::d_initialCavityVolume;
    using SurfacePressureModel<DataTypes>::d_maxPressure;
    using SurfacePressureModel<DataTypes>::d_minPressure;
    using SurfacePressureModel<DataTypes>::d_eqPressure;
    using SurfacePressureModel<DataTypes>::d_maxVolumeGrowth;
    using SurfacePressureModel<DataTypes>::d_minVolumeGrowth;
    using SurfacePressureModel<DataTypes>::d_pressure ;
    using SurfacePressureModel<DataTypes>::d_volumeGrowth ;
    using SurfacePressureModel<DataTypes>::d_eqVolumeGrowth ;
    using SurfacePressureModel<DataTypes>::d_maxVolumeGrowthVariation ;
    using SurfacePressureModel<DataTypes>::m_constraintId;
    ////////////////////////////////////////////////////////////////////////////

private:
    void updateConstraint();

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using Equality<DataTypes>::m_hasDeltaEqual ;
    using Equality<DataTypes>::m_hasLambdaEqual ;
    using Equality<DataTypes>::m_deltaEqual ;
    using Equality<DataTypes>::m_lambdaEqual ;
    using Equality<DataTypes>::m_nbLines ;
    ////////////////////////////////////////////////////////////////////////////

};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#if !defined(SOFTROBOTS_INVERSE_SURFACEPRESSUREEQUALITY_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API SurfacePressureEquality<sofa::defaulttype::Vec3Types>;
#endif

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_SURFACEPRESSUREEQUALITY_H
