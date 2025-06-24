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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_VOLUMEEEFFECTOR_H
#define SOFA_COMPONENT_CONSTRAINTSET_VOLUMEEEFFECTOR_H

#include <SoftRobots.Inverse/component/behavior/Effector.h>
#include <SoftRobots/component/constraint/model/SurfacePressureModel.h>


namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::core::behavior::Effector;
using sofa::helper::ReadAccessor;
using sofa::core::VecCoordId;
using sofa::core::ConstraintParams;


/**
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/
template< class DataTypes >
class VolumeEffector : public Effector<DataTypes> , public SurfacePressureModel<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(VolumeEffector,DataTypes), SOFA_TEMPLATE(Effector,DataTypes));

    typedef typename DataTypes::Coord Coord;
    typedef typename Coord::value_type Real;
    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;


public:

    using SurfacePressureModel<DataTypes>::d_volumeGrowth;

    VolumeEffector(MechanicalState* object=nullptr);
    ~VolumeEffector() override;

    ////////////////////////// Inherited from BaseObject ////////////////////
    void init() override;
    /////////////////////////////////////////////////////////////////////////

    //////////////// Inherited from SoftRobotsConstraint ///////////////
    void getConstraintViolation(const ConstraintParams* cParams,
                                BaseVector *resV,
                                const BaseVector *Jdx) override;
    /////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.

    using SurfacePressureModel<DataTypes>::m_constraintId ;
    using SurfacePressureModel<DataTypes>::d_cavityVolume ;
    using SurfacePressureModel<DataTypes>::m_state ;
    using SurfacePressureModel<DataTypes>::d_initialCavityVolume ;
    using SurfacePressureModel<DataTypes>::getCavityVolume;

    using SurfacePressureModel<DataTypes>::d_eqPressure;
    using SurfacePressureModel<DataTypes>::d_eqVolumeGrowth;
    using SurfacePressureModel<DataTypes>::d_minPressure;
    using SurfacePressureModel<DataTypes>::d_maxPressure;
    using SurfacePressureModel<DataTypes>::d_maxPressureVariation;
    using SurfacePressureModel<DataTypes>::d_minVolumeGrowth;
    using SurfacePressureModel<DataTypes>::d_maxVolumeGrowth;
    using SurfacePressureModel<DataTypes>::d_maxVolumeGrowthVariation;

    using Effector<DataTypes>::getTarget;
    ////////////////////////////////////////////////////////////////////////////


    /////////////// Inherited from BaseSoftRobotsConstraint ////////////////
    virtual void storeResults(type::vector<double> &delta) override;
    ///////////////////////////////////////////////////////////////////////////

protected:

    Data<Real> d_desiredVolume;

};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#if !defined(SOFTROBOTS_INVERSE_VOLUMEEFFECTOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API VolumeEffector<sofa::defaulttype::Vec3Types>;
#endif

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_VOLUMEACTUATOR_H
