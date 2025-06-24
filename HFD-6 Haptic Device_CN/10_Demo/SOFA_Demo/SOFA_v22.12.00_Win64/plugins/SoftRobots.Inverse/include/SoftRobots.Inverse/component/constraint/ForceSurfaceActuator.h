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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_FORCESURFACEACTUATOR_H
#define SOFA_COMPONENT_CONSTRAINTSET_FORCESURFACEACTUATOR_H

#include <SoftRobots.Inverse/component/behavior/Actuator.h>
#include <sofa/core/topology/BaseMeshTopology.h>

#include <SoftRobots.Inverse/component/config.h>

namespace sofa
{

namespace component
{

namespace constraintset
{
    using sofa::core::behavior::Actuator;
    using sofa::core::topology::BaseMeshTopology;
    using sofa::core::visual::VisualParams;
    using sofa::core::ConstraintParams;
    using sofa::linearalgebra::BaseVector;
    using sofa::helper::ReadAccessor;
    using sofa::core::ConstVecCoordId;

/**
 * This component is used to solve an effector constraint by applying a force on a given point of a model.
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/
template< class DataTypes >
class ForceSurfaceActuator : public Actuator<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(ForceSurfaceActuator,DataTypes), SOFA_TEMPLATE(Actuator,DataTypes));

    typedef typename DataTypes::VecCoord                    VecCoord;
    typedef typename DataTypes::VecDeriv                    VecDeriv;
    typedef typename DataTypes::Coord                       Coord;
    typedef typename DataTypes::Deriv                       Deriv;
    typedef typename DataTypes::MatrixDeriv                 MatrixDeriv;
    typedef typename Coord::value_type                      Real;

    typedef typename core::topology::BaseMeshTopology::Triangle      Triangle;
    typedef typename core::topology::BaseMeshTopology::Quad          Quad;
    typedef typename core::topology::BaseMeshTopology::Edge          Edge;

    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename DataTypes::MatrixDeriv::RowIterator    MatrixDerivRowIterator;
    typedef Data<VecCoord>                                  DataVecCoord;
    typedef Data<VecDeriv>                                  DataVecDeriv;
    typedef Data<MatrixDeriv>                               DataMatrixDeriv;


public:
    ForceSurfaceActuator(MechanicalState* = nullptr);
    ~ForceSurfaceActuator() override;

    /////////////// Inherited from BaseObject ////////////////////
    void init() override;
    void reinit() override;
    void draw(const VisualParams* vparams) override;
    /////////////////////////////////////////////////////////////

    ///////// Inherited from SoftRobotsConstraint ////////////
    void buildConstraintMatrix(const ConstraintParams* cParams ,
                               DataMatrixDeriv &cMatrix,
                               unsigned int &cIndex,
                               const DataVecCoord &x) override;

    void getConstraintViolation(const ConstraintParams* cParams ,
                                BaseVector *resV,
                                const BaseVector *Jdx) override;
    /////////////////////////////////////////////////////////////////////////

    /////////////// Inherited from BaseSoftRobotsConstraint ////////////////
    void storeResults(type::vector<double> &lambda,
                      type::vector<double> &delta) override;
    /////////////////////////////////////////////////////////////


protected:

    Data<VecCoord>                      d_centers;
    VecCoord                            m_initialCenters;
    Data<type::vector<Real>>          d_radii;
    Data<VecDeriv>                      d_directions;
    Data<bool>                          d_updateNormals;

    Data<type::vector<Triangle>>      d_triangles;
    Data<type::vector<Quad>>          d_quads;
    Data<type::vector<Coord>>         d_positions;
    type::vector<Edge>                m_edges;

    Data<Real>                          d_maxForce;
    Data<Real>                          d_minForce;
    Data<Real>                          d_maxForceVariation;

    Data<Real>                          d_maxDisplacement;
    Data<Real>                          d_minDisplacement;

    Data<type::vector<Real>>          d_force;
    Data<type::vector<Real>>          d_displacement;

    Data<bool>                          d_drawForce;
    Data<bool>                          d_drawSphere;
    Data<bool>                          d_drawSurface;
    Data<Real>                          d_visuScale;

    bool                                m_useNormals{false};

    type::vector<type::vector<unsigned int>>        m_pointsInSphereId;
    type::vector<type::vector<unsigned int>>        m_trianglesInSpheresId;
    type::vector<type::vector<unsigned int>>        m_quadsInSpheresId;
    type::vector<type::vector<unsigned int>>        m_edgesInSpheresId;
    type::vector<type::vector<Real>>                m_ratios;




    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using Actuator<DataTypes>::m_state ;
    using Actuator<DataTypes>::m_constraintId ;
    ////////////////////////////////////////////////////////////////////////////


    void initLimit();
    void initData();
    void updateLimit();
    void updateCenter();

    bool isPointInSphere(unsigned int pointId, unsigned int sphereId);
    bool isTriangleInSphere(unsigned int triangleId, unsigned int sphereId);
    bool isQuadInSphere(unsigned int quadId, unsigned int sphereId);
    bool isEdgeInSphere(unsigned int edgeId, unsigned int sphereId);

    bool isIndexInPointsList(unsigned int index, unsigned int sphereId);

    void computeSurfaces();
    void computePointsInSpheres();
    void computeNormals();
    void computeEdges();

    void drawForces(const VisualParams* vparams);
    void drawSpheres(const VisualParams* vparams);
    void drawSurfaces(const VisualParams* vparams);
    void drawTriangles(const VisualParams* vparams);
    void drawLines(const VisualParams* vparams);

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using Actuator<DataTypes>::m_lambdaMax ;
    using Actuator<DataTypes>::m_lambdaMin ;
    using Actuator<DataTypes>::m_hasLambdaMax ;
    using Actuator<DataTypes>::m_hasLambdaMin ;
    using Actuator<DataTypes>::m_deltaMax ;
    using Actuator<DataTypes>::m_deltaMin ;
    using Actuator<DataTypes>::m_hasDeltaMax ;
    using Actuator<DataTypes>::m_hasDeltaMin ;
    using Actuator<DataTypes>::m_nbLines ;
    ////////////////////////////////////////////////////////////////////////////

};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#if !defined(SOFTROBOTS_INVERSE_FORCESURFACEACTUATOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API ForceSurfaceActuator<sofa::defaulttype::Vec3Types>;
#endif

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_FORCESURFACEACTUATOR_H
