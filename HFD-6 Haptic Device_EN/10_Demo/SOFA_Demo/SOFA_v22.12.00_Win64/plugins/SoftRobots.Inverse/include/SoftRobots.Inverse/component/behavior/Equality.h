/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 INRIA, USTL, UJF, CNRS, MGH                    *
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
*                              SOFA :: Framework                              *
*                                                                             *
* Authors: The SOFA Team (see Authors.txt)                                    *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#include <SoftRobots/component/behavior/SoftRobotsConstraint.h>

#include <SoftRobots.Inverse/component/config.h>

namespace sofa::core::behavior
{

/**
 *  \brief This class specifies the type of the constraint as Equality.
 */

template<class DataTypes>
class Equality : virtual public SoftRobotsConstraint<DataTypes>
{
public:

    SOFA_CLASS(SOFA_TEMPLATE(Equality,DataTypes), SOFA_TEMPLATE(SoftRobotsConstraint,DataTypes));

    Equality(MechanicalState<DataTypes> *mm = nullptr);
    ~Equality() override;

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using SoftRobotsBaseConstraint::m_constraintType ;
    using SoftRobotsBaseConstraint::EQUALITY ;
    ////////////////////////////////////////////////////////////////////////////
};

// Force template specialization for the most common sofa float related type.
// This goes with the extern template declaration in the .h. Declaring extern template
// avoid the code generation of the template for each compilation unit.
// see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#if !defined(SOFTROBOTS_INVERSE_EQUALITY_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API Equality<sofa::defaulttype::Vec3Types>;
extern template class SOFA_SOFTROBOTS_INVERSE_API Equality<sofa::defaulttype::Vec2Types>;
extern template class SOFA_SOFTROBOTS_INVERSE_API Equality<sofa::defaulttype::Vec1Types>;
extern template class SOFA_SOFTROBOTS_INVERSE_API Equality<sofa::defaulttype::Rigid3Types>;
#endif

} // namespace sofa::core::behavior
