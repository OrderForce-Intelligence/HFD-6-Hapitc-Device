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
#ifndef SOFA_HELPER_NLCPSOLVER_H
#define SOFA_HELPER_NLCPSOLVER_H

#include <sofa/helper/system/thread/CTime.h>
#include <sofa/type/vector.h>
#include <sofa/helper/LCPcalc.h>


// NLCP solver for friction contact
// Mainly based on LCPCalc implementation
// But with different discretisation of Coulomb's friction cone


namespace sofa
{

namespace helper
{

class NLCPSolver
{

protected:
    bool m_allowSliding;

public:
    NLCPSolver(){}
    ~NLCPSolver(){}

    int solve(int dim, double *dfree, double**W, double *f, double mu, double tol, int numItMax,
              bool useInitialF, bool verbose = false, double minW=0.0, double maxF=0.0,
              type::vector<double>* residuals = NULL, type::vector<double>* violations = NULL);

    void setAllowSliding(bool allowSliding) {m_allowSliding=allowSliding;}
};

class NLCPSolverMatrix33
{

public:
    double m_w[6];
    bool m_stored;

public:
    NLCPSolverMatrix33() {m_stored=false;}
    ~NLCPSolverMatrix33() {}

    void storeW(double &w11, double &w12, double &w13, double &w22, double &w23, double &w33);
    void GSState(double &mu, double &dn, double &dt, double &ds, double &fn, double &ft, double &fs, bool allowSliding);
};

} // namespace helper

} // namespace sofa

#endif
