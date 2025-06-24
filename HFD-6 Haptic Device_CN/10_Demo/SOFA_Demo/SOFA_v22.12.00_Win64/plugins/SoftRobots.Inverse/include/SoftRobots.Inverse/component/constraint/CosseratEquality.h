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
#pragma once

#include <SoftRobots.Inverse/component/behavior/Equality.h>
#include <SoftRobots/component/constraint/model/CableModel.h>
#include <SoftRobots.Inverse/component/config.h>


namespace sofa::component::constraintset
{

using sofa::core::behavior::Equality;
using sofa::helper::ReadAccessor;
using sofa::core::VecCoordId;
using sofa::core::ConstraintParams;
using sofa::core::visual::VisualParams;
using sofa::linearalgebra::BaseVector;
using sofa::core::behavior::SoftRobotsBaseConstraint;
using sofa::core::behavior::SoftRobotsConstraint;


template< class DataTypes >
class CosseratEquality : public Equality<DataTypes>, public CableModel<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(CosseratEquality,DataTypes), SOFA_TEMPLATE(Equality,DataTypes));

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

public:
    CosseratEquality(MechanicalState* object = nullptr);
    ~CosseratEquality() override;

    ////////////////////////// Inherited from BaseObject ////////////////////
    void init() override;
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
    ///
    using CableModel<DataTypes>::d_maxDispVariation ;
    using CableModel<DataTypes>::d_maxNegativeDisplacement ;
    using CableModel<DataTypes>::d_maxPositiveDisplacement ;
    using CableModel<DataTypes>::d_eqDisplacement ;
    using CableModel<DataTypes>::d_maxForce ;
    using CableModel<DataTypes>::d_minForce ;
    using CableModel<DataTypes>::d_eqForce ;
    using CableModel<DataTypes>::d_color ;

    using Equality<DataTypes>::m_hasDeltaEqual ;
    using Equality<DataTypes>::m_deltaEqual ;

    using Equality<DataTypes>::m_hasLambdaEqual;
    using Equality<DataTypes>::m_lambdaEqual ;

    using SoftRobotsBaseConstraint::m_constraintId ;
    using SoftRobotsBaseConstraint::m_nbLines ;
    using Equality<DataTypes>::d_componentState ;

    using SoftRobotsConstraint<DataTypes>::m_state ;
    ////////////////////////////////////////////////////////////////////////////


protected:

    Data<type::vector<Real>> d_force;
    Data<type::vector<Real>> d_displacement;

    Data<Real>      d_constrainAtTime;

    void updateConstraint();
};

//// Declares template as extern to avoid the code generation of the template for
//// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
///
/// Because of template specialization in CosseratEquality.cpp we need to prevent the
/// following when we are compiling CosseratEquality.cpp. This is done
/// By setting this macro SOFTROBOTSINVERSE_CONSTRAINT_COSSERATEQUALITY_CPP
/// before including this file.
#ifndef SOFTROBOTSINVERSE_CONSTRAINT_COSSERATEQUALITY_CPP
extern template class SOFA_SOFTROBOTS_INVERSE_API CosseratEquality<sofa::defaulttype::Vec3Types>;
#endif // SOFTROBOTSINVERSE_CONSTRAINT_COSSERATEQUALITY_CPP

} // namespace sofa::component::constraintset

// SOFA_COMPONENT_CONSTRAINTSET_CosseratEquality_H
