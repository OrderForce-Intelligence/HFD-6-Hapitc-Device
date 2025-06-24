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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_QPINVERSEPROBLEMSOLVER_H
#define SOFA_COMPONENT_CONSTRAINTSET_QPINVERSEPROBLEMSOLVER_H

#include <sofa/component/constraint/lagrangian/solver/ConstraintSolverImpl.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/core/behavior/ConstraintSolver.h>
#include <sofa/core/behavior/BaseConstraintCorrection.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/simulation/TaskScheduler.h>
#include <sofa/simulation/InitTasks.h>
#include <sofa/helper/map.h>

#include <SoftRobots/component/behavior/SoftRobotsBaseConstraint.h>
#include <SoftRobots.Inverse/component/solver/modules/QPInverseProblemImpl.h>
#include <SoftRobots.Inverse/component/config.h>

using sofa::core::objectmodel::KeypressedEvent ;

namespace sofa
{

namespace component
{

namespace constraintset
{

namespace _qpinverseproblemsolver_
{

using helper::system::thread::CTime;
using type::vector;
using core::behavior::ConstraintSolver;
using core::behavior::BaseConstraintCorrection;
using sofa::core::MultiVecId;
using core::ConstraintParams;
using sofa::core::ExecParams;
using std::map;
using std::string;
using sofa::simulation::Node;
using sofa::core::MultiVecDerivId;

/**
 * This component solves an inverse problem set by actuator and effector constraints. The method
 * is based on the formulation of a quadratic program (QP).
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/

class SOFA_SOFTROBOTS_INVERSE_API QPInverseProblemSolver : public constraint::lagrangian::solver::ConstraintSolverImpl
{
public:
    SOFA_CLASS(QPInverseProblemSolver, ConstraintSolver);

    typedef vector<BaseConstraintCorrection*> list_cc;
    typedef vector<list_cc> VecListcc;

public:
    QPInverseProblemSolver();
    ~QPInverseProblemSolver() override;

    ////////////////////// Inherited from BaseObject /////////////////////////////////
    void init() override;
    void reinit() override;
    void cleanup() override;
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////// Inherited from ConstraintSolver ///////////////////////////
    bool prepareStates(const ConstraintParams * cParams,
                       MultiVecId res1,
                       MultiVecId res2=MultiVecId::null()) override;

    bool buildSystem(const ConstraintParams * cParams,
                     MultiVecId res1,
                     MultiVecId res2=MultiVecId::null()) override;

    void rebuildSystem(double massFactor,
                       double forceFactor) override;

    bool solveSystem(const ConstraintParams* cParams,
                     MultiVecId res1, MultiVecId res2=MultiVecId::null()) override;

    bool applyCorrection(const ConstraintParams * cParams,
                         MultiVecId res1,
                         MultiVecId res2=MultiVecId::null()) override;

    void computeResidual(const ExecParams* params) override;

    void removeConstraintCorrection(BaseConstraintCorrection *s) override;

    MultiVecDerivId getLambda() const override
    {
        return m_lambdaId;
    }

    MultiVecDerivId getDx() const override
    {
        return m_dxId;
    }


    ////////////////////// Inherited from ConstraintSolverImpl ////////////////////////
    constraint::lagrangian::solver::ConstraintProblem* getConstraintProblem() override;

    void lockConstraintProblem(BaseObject *from,
                               constraint::lagrangian::solver::ConstraintProblem* CP1,
                               constraint::lagrangian::solver::ConstraintProblem* CP2=nullptr) override;
    /////////////////////////////////////////////////////////////////////////////////


    Data<bool>      d_displayTime;
    Data<bool>      d_multithreading;
    Data<bool>      d_reverseAccumulateOrder;

    Data<int>       d_countdownFilterStartPerturb;
    Data<bool>      d_saveMatrices;

    Data<int>       d_maxIterations;
    Data<double>    d_tolerance;
    Data<double>    d_responseFriction;

    Data<double>    d_epsilon;
    Data<bool>      d_actuatorsOnly;
    Data<bool>      d_allowSliding;
    Data<map <string, vector<SReal> > > d_graph;
    Data<double>    d_minContactForces;
    Data<double>    d_maxContactForces;
    Data<SReal >    d_objective;

protected:

    MultiVecDerivId m_lambdaId;
    MultiVecDerivId m_dxId;

    QPInverseProblemImpl *m_CP1, *m_CP2, *m_CP3;
    QPInverseProblemImpl *m_lastCP, *m_currentCP;
    vector<BaseConstraintCorrection*> m_constraintsCorrections;
    vector<char> m_isConstraintCorrectionActive;

    Node *m_context;

    CTime m_timer;
    CTime m_timerTotal;

    double m_time;
    double m_timeTotal;
    double m_timeScale;

    virtual void createProblems();
    void deleteProblems();


private:
    void accumulateConstraint(const ConstraintParams *cParams, unsigned int & nbLinesTotal);
    void setConstraintProblemSize(const unsigned int &nbLinesTotal);
    void computeConstraintViolation(const ConstraintParams *cParams);
    void getConstraintCorrectionState();
    void buildCompliance(const ConstraintParams *cParams);

    class ComputeComplianceTask : public simulation::CpuTask
    {
    public:
        ComputeComplianceTask(simulation::CpuTask::Status* status): CpuTask(status) {}
        ~ComputeComplianceTask() override {}

        MemoryAlloc run() final {
            cc->addComplianceInConstraintSpace(&cparams, &W);
            return MemoryAlloc::Stack;
        }

        void set(core::behavior::BaseConstraintCorrection* _cc, core::ConstraintParams _cparams, int dim){
            cc = _cc;
            cparams = _cparams;
            W.resize(dim,dim);
        }

    private:
        core::behavior::BaseConstraintCorrection* cc;
        sofa::linearalgebra::LPtrFullMatrix<double> W;
        core::ConstraintParams cparams;
        friend class QPInverseProblemSolver;
    };
};

}

using _qpinverseproblemsolver_::QPInverseProblemSolver;

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif
