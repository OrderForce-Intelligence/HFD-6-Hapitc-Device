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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_QPINVERSEPROBLEMIMPL_H
#define SOFA_COMPONENT_CONSTRAINTSET_QPINVERSEPROBLEMIMPL_H

#include <Eigen/Core>
#include <qpOASES/Types.hpp>
#include <qpOASES/QProblem.hpp>
#include <SoftRobots/component/behavior/SoftRobotsBaseConstraint.h>

#include "ConstraintHandler.h"
#include "QPInverseProblem.h"

#include <SoftRobots.Inverse/component/config.h>


namespace sofa
{

namespace component
{

namespace constraintset
{

namespace _qpinverseproblemimpl_
{

using type::vector;
using qpOASES::real_t;


class SOFA_SOFTROBOTS_INVERSE_API QPInverseProblemImpl : public QPInverseProblem
{

public:

    typedef Eigen::Ref<Eigen::MatrixXd> RefMat;
    typedef const Eigen::Ref<const Eigen::MatrixXd> ConstRefMat;
    typedef Eigen::Ref<Eigen::VectorXd> RefVec;
    typedef const Eigen::Ref<const Eigen::VectorXd> ConstRefVec;

    QPInverseProblemImpl();
    virtual ~QPInverseProblemImpl();

    void init();
    void solve(double &objective, int &iterations);
    void setMinContactForces(const double& minContactForces) {m_qpCParams->minContactForces = minContactForces; m_qpCParams->hasMinContactForces = true;}
    void setMaxContactForces(const double& maxContactForces) {m_qpCParams->maxContactForces = maxContactForces; m_qpCParams->hasMaxContactForces = true;}

protected:

    ConstraintHandler* m_constraintHandler;
    ConstraintHandler::QPConstraintParams* m_qpCParams;

    // Utils to prevent cycling in pivot algorithm
    vector<int>   m_currentSequence;
    vector<int>   m_previousSequence;
    vector<int>   m_sequence;

    int m_iteration{0};
    int m_step{0};


    void computeEnergyWeight(double& weight);
    void buildQPMatrices();


    void solveWithContact(vector<double>& result, double &objective, int &iterations);
    void solveContacts(vector<double>& res);
    void solveInverseProblem(double &objective,
                             vector<double> &result,
                             vector<double> &dual);

    void updateOASESMatrices(real_t * Q, real_t * c, real_t * l, real_t * u,
                             real_t * A, real_t * bl, real_t * bu);

    void updateLambda(const vector<double>& x);
    bool isFeasible(const vector<double>& x);

    bool checkAndUpdatePivot(const vector<double>&result, const vector<double>&dual);
    bool isCycling(const int pivot);
    double getDelta(const vector<double> &result, const int& index);
    bool isIn(const vector<int> list, const int elem);
    std::string getContactsState();


private:
    qpOASES::QProblem getNewQProblem(int &nWSR);

};

}

using _qpinverseproblemimpl_::QPInverseProblemImpl;

}
}
}

#endif
