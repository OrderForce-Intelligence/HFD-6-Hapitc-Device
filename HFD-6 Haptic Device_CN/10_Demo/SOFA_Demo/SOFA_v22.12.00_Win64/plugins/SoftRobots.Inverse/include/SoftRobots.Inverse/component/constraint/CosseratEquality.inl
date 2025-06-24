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
#pragma once

#include <sofa/core/visual/VisualParams.h>
#include <sofa/type/Vec.h>
#include "CosseratEquality.h"

namespace sofa::component::constraintset
{

using sofa::core::objectmodel::ComponentState ;

using sofa::core::visual::VisualParams;
using sofa::linearalgebra::BaseVector;
using sofa::helper::ReadAccessor;
using sofa::helper::rabs;
using sofa::type::Vec;

template<class DataTypes>
CosseratEquality<DataTypes>::CosseratEquality(MechanicalState* object)
    : Equality<DataTypes>(object)
    , CableModel<DataTypes>(object)
    , d_force(initData(&d_force, "force","Output force."))
    , d_displacement(initData(&d_displacement, "displacement"," The displacement "))
    , d_constrainAtTime(initData(&d_constrainAtTime,Real(0.0), "constrainAtTime",
                                 "No constraints will be applied before this time. \n"
                                 "Example of use: to allow the cable to reach an \n"
                                 "initial configuration before optimizing."))
{
    m_hasDeltaEqual = true;
    d_maxForce.setDisplayed(false);
    d_minForce.setDisplayed(false);
    d_maxPositiveDisplacement.setDisplayed(false);
    d_maxNegativeDisplacement.setDisplayed(false);
    d_maxDispVariation.setDisplayed(false);

    // CosseratEquality of dimension 1
    m_lambdaEqual.resize(1);
    m_deltaEqual.resize(1);
}


template<class DataTypes>
CosseratEquality<DataTypes>::~CosseratEquality()
{
}

template<class DataTypes>
void CosseratEquality<DataTypes>::init()
{
    CableModel<DataTypes>::init();

    helper::WriteAccessor<Data<type::vector<double>>> force = d_force;
    helper::WriteAccessor<Data<type::vector<double>>> displacement = d_displacement;

    ///TODO need to be changed
    size_t sz = 2 * (m_state->getSize() - 1) + 3;
    force.resize(sz);
    displacement.resize(sz);

    for (size_t i = 0;i < sz ; i++ ) {
        force[i] = 0.0;
        displacement[i] = 0.0;
    }
    updateConstraint();
}


template<class DataTypes>
void CosseratEquality<DataTypes>::reinit()
{
    CableModel<DataTypes>::reinit();
    updateConstraint();
}

template<class DataTypes>
void CosseratEquality<DataTypes>::reset()
{
    updateConstraint();
}


template<class DataTypes>
void CosseratEquality<DataTypes>::updateConstraint()
{
    Real time = this->getContext()->getTime();
    if(time < d_constrainAtTime.getValue())
        return;

    ReadAccessor<Data<Real>> maxEqDisplacement = d_eqDisplacement;
    ReadAccessor<Data<Real>> eqForce = d_eqForce;

    if(d_eqForce.isSet())
    {
        m_hasLambdaEqual = true;
        m_lambdaEqual[0] = eqForce;
    }

    if(d_eqDisplacement.isSet())
    {
        m_hasDeltaEqual = true;
        m_deltaEqual[0] = maxEqDisplacement;
    }
}


template<class DataTypes>
void CosseratEquality<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                        DataMatrixDeriv &cMatrix,
                                                        unsigned int &cIndex,
                                                        const DataVecCoord &x)
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return ;

    SOFA_UNUSED(cParams);
    MatrixDeriv& matrix = *cMatrix.beginEdit();
    VecCoord positions = x.getValue();
    m_constraintId= cIndex;

    unsigned int szPos = positions.size();
    for (unsigned int i=0; i<szPos; i++)
    {
        if(i < szPos - 1){
            MatrixDerivRowIterator c_it = matrix.writeLine(cIndex);
            c_it.addCol(i, Coord(0,1,0));
            MatrixDerivRowIterator c_it_1 = matrix.writeLine(cIndex+1);
            c_it_1.addCol(i, Coord(0,0,1));
            cIndex +=2;
        }else{
            MatrixDerivRowIterator c_it = matrix.writeLine(cIndex);
            c_it.addCol(i, Coord(1,0,0));
            MatrixDerivRowIterator c_it_1 = matrix.writeLine(cIndex+1);
            c_it_1.addCol(i, Coord(0,1,0));
            MatrixDerivRowIterator c_it_2 = matrix.writeLine(cIndex+2);
            c_it_2.addCol(i, Coord(0,0,1));
            cIndex +=3;
        }

    }
    cMatrix.endEdit();
    m_nbLines = cIndex - m_constraintId;
}


template<class DataTypes>
void CosseratEquality<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                         BaseVector *resV,
                                                         const BaseVector *Jdx)
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return ;

    SOFA_UNUSED(cParams);
    ReadAccessor<Data<VecCoord>> positions = m_state->readPositions();

    unsigned int szPos = positions.size();
    if(Jdx->size()==0){
        for (size_t i = 0; i < szPos; i++){
            if( i < szPos-1){
                Real dfree1 =  positions[i][1];
                Real dfree2 =  positions[i][2];

                resV->set(m_constraintId + 2*i   , dfree1);
                resV->set(m_constraintId + 2*i +1, dfree2);
            }else{
                Real dfree0 =  positions[i][0];
                Real dfree1 =  positions[i][1];
                Real dfree2 =  positions[i][2];

                resV->set(m_constraintId + 2*i   , dfree0);
                resV->set(m_constraintId + 2*i +1, dfree1);
                resV->set(m_constraintId + 2*i +2, dfree2);
            }
        }
    }else{
        for (size_t i = 0; i < szPos; i++){
            if( i < szPos-1){
                Real dfree1 = Jdx->element(2*i)   + positions[i][1];
                Real dfree2 = Jdx->element(2*i+1) + positions[i][2];

                resV->set(m_constraintId + 2*i   , dfree1);
                resV->set(m_constraintId + 2*i +1, dfree2);
            }else{
                Real dfree0 = Jdx->element(2*i)   + positions[i][0];
                Real dfree1 = Jdx->element(2*i+1) + positions[i][1];
                Real dfree2 = Jdx->element(2*i+2) + positions[i][2];
                resV->set(m_constraintId + 2*i   , dfree0);
                resV->set(m_constraintId + 2*i +1, dfree1);
                resV->set(m_constraintId + 2*i +2, dfree2);
            }
        }
    }
}

template<class DataTypes>
void CosseratEquality<DataTypes>::storeResults(type::vector<double> &lambda,
                                               type::vector<double> &delta)
{
    helper::WriteAccessor<Data<type::vector<double>>> force = d_force;
    helper::WriteAccessor<Data<type::vector<double>>> displacement = d_displacement;
    size_t sz = lambda.size();
    force.resize(sz);
    displacement.resize(delta.size());

    for (size_t i = 0;i < sz ; i++ ) {
        force[i] = lambda[i];
        displacement[i] = delta[i];
    }
    updateConstraint();
}


template<class DataTypes>
void CosseratEquality<DataTypes>::draw(const VisualParams* vparams)
{
    SOFA_UNUSED(vparams);
    if(d_componentState.getValue() != ComponentState::Valid)
        return ;
}

} // namespace sofa::component::constraintset
