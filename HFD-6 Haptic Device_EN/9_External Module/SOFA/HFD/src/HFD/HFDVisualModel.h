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
#ifndef SOFA_HFD_VISUALMODEL_H
#define SOFA_HFD_VISUALMODEL_H

//HFD include
#include <HFD/config.h>
#include <HFD/HFDDriver.h>
#include <sofa/gl/component/rendering3d/OglModel.h>
#include <sofa/component/io/mesh/MeshOBJLoader.h>

#include <sofa/type/Vec.h>
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/defaulttype/RigidTypes.h>

//Visualization
#include <sofa/component/mapping/nonlinear/RigidMapping.h>
#include <sofa/component/statecontainer/MechanicalObject.h>
#include <sofa/simulation/Node.h>

namespace sofa 
{

namespace component 
{

namespace controller
{

using namespace sofa::defaulttype;

/**
* Class used by HFDDriver to display the HFD device position and motion using visual models in the 3D scene.
*/
class SOFA_HFD_API HFDVisualModel
{
public:
    typedef RigidTypes::Coord Coord;
    typedef RigidTypes::VecCoord VecCoord;

    struct VisualComponent
    {
        simulation::Node::SPtr node;
        sofa::component::io::mesh::MeshOBJLoader::SPtr loader;
        sofa::gl::component::rendering3d::OglModel::SPtr visu;
        sofa::component::mapping::nonlinear::RigidMapping< Rigid3Types , Vec3Types  >::SPtr mapping;
    };


    HFDVisualModel();
	virtual ~HFDVisualModel();

    /// Main Method to init the visual component tree of OGLModels. Called by HFD InitDevice() if drawVisual is on.
    void initDisplay(sofa::simulation::Node::SPtr node, const std::string& _deviceName, double _scale);

    /// Method to update the visualNode using the current device position and the angles of the different node of the device. Updated by HFD UpdatePosition()
    void updateDisplay(const HFDDriver::Coord& posDevice, HFDDriver::SHDdouble angle1[3], HFDDriver::SHDdouble angle2[3]);

    /// Method called by HFD Draw method to display the HFD OglModel
    void drawDevice(bool button1Status = false, bool button2Status = false);

    /// Get status if visualisation is activated
    bool isDisplayActivated() const { return m_displayActived; }
    /// Activate or not the visualisation
    void activateDisplay(bool value);

    /// Get status if visualisation is init
    bool isDisplayInitiate() const { return m_initDisplayDone; }

protected:
    /// variable pour affichage graphique
    enum
    {
        VN_stylus = 0,
        VN_joint2 = 1,
        VN_joint1 = 2,
        VN_arm2 = 3,
        VN_arm1 = 4,
        VN_joint0 = 5,
        VN_base = 6,
        NVISUALNODE = 7
    };
    VisualComponent visualNode[NVISUALNODE];
    static const char* visualNodeNames[NVISUALNODE];
    static const char* visualNodeFiles[NVISUALNODE];
    simulation::Node::SPtr m_omniVisualNode;
    component::statecontainer::MechanicalObject<sofa::defaulttype::Rigid3dTypes>::SPtr rigidDOF;

    VecCoord m_posDeviceVisu; ///< position of the hpatic devices for rendering. first pos is equal to d_posDevice

private:
    bool m_displayActived; ///< Internal boolean to detect activation switch of the draw
    bool m_initDisplayDone; ///< Internal boolean activated only if visu initialization done without return
    double m_scale;
};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_HFD_VISUALMODEL_H
