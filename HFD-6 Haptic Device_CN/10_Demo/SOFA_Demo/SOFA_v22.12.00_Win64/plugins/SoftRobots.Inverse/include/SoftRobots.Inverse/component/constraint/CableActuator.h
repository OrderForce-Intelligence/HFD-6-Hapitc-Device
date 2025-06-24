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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_CABLEACTUATOR_H
#define SOFA_COMPONENT_CONSTRAINTSET_CABLEACTUATOR_H

#include <SoftRobots.Inverse/component/behavior/Actuator.h>
#include <SoftRobots/component/constraint/model/CableModel.h>

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


/**
 * This component simulates a force exerted by a cable to solve an effector constraint.
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/
template< class DataTypes >
class CableActuator : public Actuator<DataTypes> , public CableModel<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(CableActuator,DataTypes), SOFA_TEMPLATE(Actuator,DataTypes));

    typedef typename DataTypes::Coord Coord;
    typedef typename Coord::value_type Real;
    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;


public:
    CableActuator(MechanicalState* object = nullptr);
    ~CableActuator() override;

    ////////////////////////// Inherited from BaseObject ////////////////////
    void init() override;
    void reinit() override;
    void reset() override;
    /////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using CableModel<DataTypes>::d_force ;
    using CableModel<DataTypes>::d_displacement ;
    using CableModel<DataTypes>::d_maxDispVariation ;
    using CableModel<DataTypes>::d_maxNegativeDisplacement ;
    using CableModel<DataTypes>::d_maxPositiveDisplacement ;
    using CableModel<DataTypes>::d_eqDisplacement ;
    using CableModel<DataTypes>::d_maxForce ;
    using CableModel<DataTypes>::d_minForce ;
    using CableModel<DataTypes>::d_eqForce ;
    using CableModel<DataTypes>::d_color ;
    using CableModel<DataTypes>::m_state ;

    using Actuator<DataTypes>::m_hasDeltaMax ;
    using Actuator<DataTypes>::m_hasDeltaMin ;
    using Actuator<DataTypes>::m_deltaMax ;
    using Actuator<DataTypes>::m_deltaMin ;

    using Actuator<DataTypes>::m_hasLambdaMax ;
    using Actuator<DataTypes>::m_hasLambdaMin ;
    using Actuator<DataTypes>::m_hasLambdaInit ;
    using Actuator<DataTypes>::m_lambdaMax ;
    using Actuator<DataTypes>::m_lambdaMin ;
    using Actuator<DataTypes>::m_lambdaInit ;
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited from BaseConstraint ////////////////
    void storeLambda(const ConstraintParams* cParams,
                     core::MultiVecDerivId res,
                     const BaseVector* lambda) override;
    /////////////////////////////////////////////////////////////////////////

    /////////////// Inherited from BaseSoftRobotsConstraint ////////////////
    void storeResults(type::vector<double> &lambda,
                      type::vector<double> &delta) override;
    ////////////////////////////////////////////////////////////////////////

protected:
    Data<Real>                  d_initForce;
    Data<Real>                  d_constrainAtTime;
    Data<bool>                  d_displayCableLimit;

    type::RGBAColor    m_color;

    void initDatas();
    void initLimit();
    void updateLimit();
    void updateVisualization();
};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#if !defined(SOFTROBOTS_INVERSE_CABLEACTUATOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API CableActuator<sofa::defaulttype::Vec3Types>;
#endif

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_CABLEACTUATOR_H
