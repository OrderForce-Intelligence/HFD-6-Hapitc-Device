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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_SLIDINGACTUATOR_H
#define SOFA_COMPONENT_CONSTRAINTSET_SLIDINGACTUATOR_H

#include <SoftRobots.Inverse/component/behavior/Actuator.h>

#include <SoftRobots.Inverse/component/config.h>

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::core::behavior::Actuator;
using sofa::helper::ReadAccessor;
using sofa::core::VecCoordId;
using sofa::core::ConstraintParams;
using sofa::core::visual::VisualParams;
using sofa::linearalgebra::BaseVector;
using sofa::core::behavior::SoftRobotsBaseConstraint;
using sofa::core::behavior::SoftRobotsConstraint;


/**
 * This component simulates a force exerted along a given direction to solve an effector.
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/
template< class DataTypes >
class SlidingActuator : public Actuator<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SlidingActuator,DataTypes), SOFA_TEMPLATE(Actuator,DataTypes));

    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename Coord::value_type Real;
    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef core::objectmodel::Data<VecCoord>		DataVecCoord;
    typedef core::objectmodel::Data<VecDeriv>		DataVecDeriv;
    typedef core::objectmodel::Data<MatrixDeriv>    DataMatrixDeriv;

    typedef type::vector<unsigned int> SetIndexArray;
    typedef Actuator<DataTypes> Inherit;

public:
    SlidingActuator(MechanicalState* object = nullptr);
    ~SlidingActuator() override;

    ////////////////////////// Inherited from BaseObject ////////////////////
    void init() override;
    void bwdInit() override;
    void reinit() override;
    void reset() override;
    void draw(const VisualParams* vparams) override;
    /////////////////////////////////////////////////////////////////////////

    //////////////// Inherited from SoftRobotsConstraint ///////////////
    void buildConstraintMatrix(const ConstraintParams* cParams,
                               DataMatrixDeriv &cMatrix,
                               unsigned int &cIndex,
                               const DataVecCoord &x) override;

    void getConstraintViolation(const ConstraintParams* cParams,
                                BaseVector *resV,
                                const BaseVector * Jdx) override;
    /////////////////////////////////////////////////////////////////////////


    /////////////// Inherited from BaseSoftRobotsConstraint /////////////
    void storeResults(type::vector<double> &lambda,
                      type::vector<double> &delta) override;
    /////////////////////////////////////////////////////////////

protected:

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using Inherit::m_hasDeltaMax ;
    using Inherit::m_hasDeltaMin ;
    using Inherit::m_deltaMax ;
    using Inherit::m_deltaMin ;

    using Inherit::m_hasLambdaMax ;
    using Inherit::m_hasLambdaMin ;
    using Inherit::m_lambdaMax ;
    using Inherit::m_lambdaMin ;

    using Inherit::m_nbLines ;
    using Inherit::m_constraintId ;
    using Inherit::d_componentState ;

    using Inherit::m_state ;
    ////////////////////////////////////////////////////////////////////////////


    Data<Real>                  d_maxPositiveDisplacement;
    Data<Real>                  d_maxNegativeDisplacement;
    Data<Real>                  d_maxDispVariation;
    Data<Real>                  d_maxForce;
    Data<Real>                  d_minForce;

    Data<Deriv>                 d_direction;
    Data<SetIndexArray>         d_indices;

    Data<double>                d_initForce;
    Data<double>                d_force;
    Data<double>                d_initDisplacement;
    Data<double>                d_displacement;

    Data<bool>                  d_showDirection;
    Data<double>                d_showVisuScale;

    void initDatas();
    void initLimit();
    void updateLimit();
    void checkIndicesRegardingState();
};

//// Declares template as extern to avoid the code generation of the template for
//// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
///
/// Because of template specialization in SlidingActuator.cpp we need to prevent the
/// following when we are compiling SlidingActuator.cpp. This is done
/// By setting this macro SOFTROBOTSINVERSE_CONSTRAINT_SLIDINGACTUATOR_NOEXTERN
/// before including this file.
#ifndef SOFTROBOTSINVERSE_CONSTRAINT_SLIDINGACTUATOR_NOEXTERN
extern template class SOFA_SOFTROBOTS_INVERSE_API SlidingActuator<sofa::defaulttype::Vec3Types>;
extern template class SOFA_SOFTROBOTS_INVERSE_API SlidingActuator<sofa::defaulttype::Rigid3Types>;
#endif // SOFTROBOTSINVERSE_CONSTRAINT_SLIDINGACTUATOR_NOEXTERN

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_SLIDINGACTUATOR_H
