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

#ifndef SOFA_COMPONENT_CONSTRAINTSET_QPMECHANICALSETCONSTRAINT_H
#define SOFA_COMPONENT_CONSTRAINTSET_QPMECHANICALSETCONSTRAINT_H

#include <sofa/component/constraint/lagrangian/solver/ConstraintSolverImpl.h>

#include <SoftRobots/component/behavior/SoftRobotsBaseConstraint.h>
#include "QPInverseProblem.h"

#include <SoftRobots.Inverse/component/config.h>

namespace sofa
{

namespace component
{

namespace constraintset
{

class SOFA_SOFTROBOTS_INVERSE_API QPMechanicalSetConstraint : public simulation::BaseMechanicalVisitor
{
public:
    QPMechanicalSetConstraint(const core::ConstraintParams* cparams,
                              core::MultiMatrixDerivId res,
                              unsigned int &contactId,
                              QPInverseProblem *currentCP) ;


    ////////////////////// Inherited from ConstraintSolverImpl ////////////////////////
    virtual Visitor::Result fwdConstraintSet(simulation::Node* node, core::behavior::BaseConstraintSet* c) ;
    virtual const char* getClassName() const ;
    virtual bool isThreadSafe() const ;
    virtual bool stopAtMechanicalMapping(simulation::Node* node, core::BaseMapping* map) ;
    /////////////////////////////////////////////////////////////////////////////////


#ifdef SOFA_DUMP_VISITOR_INFO
    void setReadWriteVectors() ;
#endif

protected:
    core::MultiMatrixDerivId m_res;
    unsigned int &m_constraintId;
    const core::ConstraintParams *m_cparams;

    QPInverseProblem* m_currentCP;

};

}
}
}

#endif
