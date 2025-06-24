/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2016 INRIA, USTL, UJF, CNRS, MGH                    *
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
#ifndef SOFA_HELPER_LCPQPSOLVER_H
#define SOFA_HELPER_LCPQPSOLVER_H


#include <sofa/type/vector.h>


// LCP solver using qpOASES QP solver
// QP formulation for solving a LCP with a symmetric matrix M.


namespace sofa
{

namespace helper
{

namespace _lcpqpsolver_
{

using type::vector;

class LCPQPSolver
{

public:

    LCPQPSolver(){}
    ~LCPQPSolver(){}

    void solve(int dim, double*q, double**M, double*res);
};

}

using _lcpqpsolver_::LCPQPSolver;

} // namespace helper

} // namespace sofa

#endif
