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
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_CONSTRAINTSET_BARYCENTRICCENTEREFFECTOR_H
#define SOFA_COMPONENT_CONSTRAINTSET_BARYCENTRICCENTEREFFECTOR_H

#include <SoftRobots.Inverse/component/behavior/Effector.h>

namespace sofa
{

namespace component
{

namespace constraintset
{
    using sofa::core::behavior::Effector ;
    using sofa::core::ConstraintParams ;
    using sofa::linearalgebra::BaseVector ;
    using sofa::core::visual::VisualParams ;
    using sofa::type::Vec ;

/**
 * The "BarycentricCenterEffector" component is used to constrain the barycenter of a model
 * to reach a desired position, by acting on chosen actuator(s).
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/
template<class DataTypes>
class BarycentricCenterEffector : public Effector<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(BarycentricCenterEffector,DataTypes),
               SOFA_TEMPLATE(Effector,DataTypes));

    typedef typename DataTypes::VecCoord    VecCoord;
    typedef typename DataTypes::VecDeriv    VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename DataTypes::Coord       Coord;
    typedef typename DataTypes::Deriv       Deriv;
    typedef typename Coord::value_type      Real;

    typedef typename sofa::core::behavior::MechanicalState<DataTypes>          MechanicalState;
    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;

    typedef Data<VecCoord>		 DataVecCoord;
    typedef Data<VecDeriv>		 DataVecDeriv;
    typedef Data<MatrixDeriv>    DataMatrixDeriv;

public:
    BarycentricCenterEffector(MechanicalState* object = nullptr);
    virtual ~BarycentricCenterEffector() override;

    ////////////////////////// Inherited from BaseObject ////////////////////
    /// According to BaseObject::init
    /// this method should be used to initialize the object during the top-down
    /// traversal of graph creation and modification,
    void init() override;

    /// According to BaseObject::reinit
    /// this method should be used to update the object when the variables used
    /// in precomputation are modified.
    void reinit() override;

    /// According to BaseObject::reset
    /// this method should be used to reset the object in its initial state.
    void reset() override;

    /// According to BaseObject::draw
    /// this method should be used to render internal data of this object,
    /// for debugging purposes.
    void draw(const VisualParams* vparams) override;
    /////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited from SoftRobotsConstraint ////////////
    void buildConstraintMatrix(const ConstraintParams* cParams,
                               DataMatrixDeriv &cMatrix,
                               unsigned int &cIndex,
                               const DataVecCoord & x) override;

    void getConstraintViolation(const ConstraintParams* cParams,
                                BaseVector *resV,
                                const BaseVector *Jdx) override;
    /////////////////////////////////////////////////////////////////////////

    /////////////// Inherited from BaseSoftRobotsConstraint ////////////////
    void storeResults(type::vector<double> &delta) override;
    ///////////////////////////////////////////////////////////////////////////

protected:

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring m_state in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// using the "this->" approach.
    using Effector<DataTypes>::m_state ;
    using Effector<DataTypes>::getContext ;
    using Effector<DataTypes>::m_nbLines ;
    using Effector<DataTypes>::m_constraintId ;
    using Effector<DataTypes>::f_listening ;
    using Effector<DataTypes>::getTarget ;

    Data<Vec<3,bool> > d_axis;
    Data<Coord >       d_effectorGoalPosition;
    Data<bool>         d_drawBarycenter;
    Data<Coord >       d_barycenter;

    Data<type::vector<double>> d_delta;

    void initData();

    void computeBarycenter();
};

template<> SOFA_SOFTROBOTS_INVERSE_API
void BarycentricCenterEffector<defaulttype::Rigid3Types>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                                                DataMatrixDeriv& cMatrix,
                                                                                unsigned int& cIndex,
                                                                                const DataVecCoord& x);

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#if !defined(SOFA_COMPONENT_CONSTRAINTSET_BARYCENTRICCENTEREFFECTOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API BarycentricCenterEffector<defaulttype::Vec3Types >;
extern template class SOFA_SOFTROBOTS_INVERSE_API BarycentricCenterEffector<defaulttype::Rigid3Types >;
#endif


} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_BarycentricCenterEffector_H
