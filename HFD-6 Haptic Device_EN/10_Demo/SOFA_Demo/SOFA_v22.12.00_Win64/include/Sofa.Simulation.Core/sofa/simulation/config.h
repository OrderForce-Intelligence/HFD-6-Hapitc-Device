/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#include <sofa/config.h>

#define SOFA_HAVE_DAG

#ifdef SOFA_BUILD_SOFA_SIMULATION_CORE
#  define SOFA_TARGET Sofa.Simulation.Core
#  define SOFA_SIMULATION_CORE_API SOFA_EXPORT_DYNAMIC_LIBRARY
#else
#  define SOFA_SIMULATION_CORE_API SOFA_IMPORT_DYNAMIC_LIBRARY
#endif

#ifdef SOFA_BUILD_SOFA_SIMULATION_CORE
#define SOFA_ATTRIBUTE_DEPRECATED_MECHANICALACCUMULATECONSTRAINT()
#else
#define SOFA_ATTRIBUTE_DEPRECATED_MECHANICALACCUMULATECONSTRAINT() \
    SOFA_ATTRIBUTE_DEPRECATED( \
    "v22.12", "v23.06", "use MechanicalBuildConstraintMatrix followed by MechanicalAccumulateMatrixDeriv")
#endif // SOFA_BUILD_SOFA_SIMULATION_CORE


#ifdef SOFA_BUILD_SOFA_SIMULATION_CORE
#define SOFA_ATTRIBUTE_DEPRECATED_STATIC_TASKSCHEDULER()
#else
#define SOFA_ATTRIBUTE_DEPRECATED_STATIC_TASKSCHEDULER() \
    SOFA_ATTRIBUTE_DEPRECATED( \
    "v22.12", "v23.06", "Use TaskSchedulerFactory instead")
#endif // SOFA_BUILD_SOFA_SIMULATION_CORE
