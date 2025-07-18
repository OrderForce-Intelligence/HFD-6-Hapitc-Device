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

#define SOFA_LINEARALGEBRA_HAVE_OPENMP 0

#define SPARSEMATRIX_CHECK false
#define SPARSEMATRIX_VERBOSE false
#define COMPRESSEDROWSPARSEMATRIX_CHECK false
#define COMPRESSEDROWSPARSEMATRIX_VERBOSE false
#define FULLMATRIX_CHECK false
#define FULLMATRIX_VERBOSE false
#define EIGEN_CHECK false


#ifdef SOFA_BUILD_SOFA_LINEARALGEBRA
#  define SOFA_TARGET Sofa.LinearAlgebra
#  define SOFA_LINEARALGEBRA_API SOFA_EXPORT_DYNAMIC_LIBRARY
#else
#  define SOFA_LINEARALGEBRA_API SOFA_IMPORT_DYNAMIC_LIBRARY
#endif

// DEPRECATION MACROS

#ifdef SOFA_BUILD_SOFA_LINEARALGEBRA
#define SOFA_MATRIXMANIPULATOR_DISABLED()
#else
#define SOFA_MATRIXMANIPULATOR_DISABLED() \
    SOFA_ATTRIBUTE_DISABLED( \
        "v22.06", "v22.12", "")
#endif // SOFA_BUILD_SOFA_LINEARALGEBRA
