import Sofa.Core
import Sofa.Simulation
import SofaRuntime
import os
import sys
import numpy as np

MeshesPath = os.path.dirname(os.path.abspath(__file__))+'/GeneratedMeshes/'

from PythonScripts.EqController import Controller
import ConstantsAccordeon as Const


def createScene(rootNode):

        rootNode.addObject('RequiredPlugin', pluginName='SofaPython3 SoftRobots SoftRobots.Inverse')
        rootNode.addObject('RequiredPlugin', name='SofaPlugins', pluginName=['SofaConstraint','SofaImplicitOdeSolver','SofaDeformable','SofaEngine','SofaLoader','SofaOpenglVisual','SofaPreconditioner','SofaSimpleFem','SofaSparseSolver'])
        rootNode.addObject('VisualStyle', displayFlags='hideWireframe showBehaviorModels hideCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields')
        rootNode.findData('gravity').value = [0, 0, 0]
        rootNode.findData('dt').value = 0.02
        rootNode.addObject('FreeMotionAnimationLoop')
        rootNode.addObject('QPInverseProblemSolver', printLog=0, epsilon=1e-3, maxIterations=1000, tolerance=1e-4)
        rootNode.addObject('BackgroundSetting', color=[1, 1, 1, 1])

        rootNode.addObject('LightManager')
        rootNode.addObject('PositionalLight', name="light1", color="0.8 0.8 0.8", position="0 60 -50")
        rootNode.addObject('PositionalLight', name="light2", color="0.8 0.8 0.8", position="0 -60 50")

        VolumetricMeshPath = MeshesPath + 'Accordeon_Volumetric.vtk'
        SurfaceMeshPath = MeshesPath + 'Accordeon_Surface.stl'

        ##########################################
        # Mechanical Model                       #
        ##########################################
        model = rootNode.addChild('model')
        model.addObject('EulerImplicitSolver', name='odesolver')
        model.addObject('ShewchukPCGLinearSolver', name='linearSolver',iterations=25, tolerance=1e-7, preconditioners="precond")
        model.addObject('SparseLDLSolver', name='precond', template='CompressedRowSparseMatrixMat3x3d')
        model.addObject('MeshVTKLoader', name='loader', filename=VolumetricMeshPath, scale3d=[1, 1, 1])
        model.addObject('MeshTopology', src='@loader', name='container')
        model.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale=4e-5)
        model.addObject('UniformMass', totalMass=0.1)
        model.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=Const.PoissonRation,  youngModulus=Const.YoungsModulus)
        model.addObject('BoxROI', name='BoxROI1', box=Const.FixedBoxCoordsBack, drawBoxes=True)
        model.addObject('RestShapeSpringsForceField', points='@BoxROI1.indices', stiffness=1e12)
        model.addObject('LinearSolverConstraintCorrection', name='GCS', solverName='precond')

        ##########################################
        # Actuation                              #
        ##########################################
        cables = model.addChild('Cables')
        cable1 = cables.addChild('Cable1')

        TotalHeight = (Const.NSegments-1) * Const.SegmentHeight
        CableHeights = np.linspace(0,TotalHeight,Const.NSegments)

        XCoords = np.ones(CableHeights.shape) * Const.Radius * 3/4
        ZCoords = np.zeros(CableHeights.shape)

        CablePositions = np.column_stack((XCoords,CableHeights,ZCoords))

        cable1.addObject('MechanicalObject', position=CablePositions.tolist())
        # Here you can set the desired cable length to reach
        cable1.addObject('CableEquality', template='Vec3d', name='CableEquality', indices=list(range(0,len(CableHeights))), pullPoint=[Const.Radius * 3/4, -10, 0], eqDisp = 5, printLog=True)
        cable1.addObject('BarycentricMapping')

        ##########################################
        # Visualization                          #
        ##########################################
        modelVisu = model.addChild('visu')
        modelVisu.addObject('MeshSTLLoader', filename=SurfaceMeshPath, name="loader")
        modelVisu.addObject('OglModel', src="@loader", scale3d=[1, 1, 1], material="Default Diffuse 1 0.8 0.8 0.8 0.95 Ambient 0 0.2 0 0 1 Specular 0 1 0 0 1 Emissive 0 1 0 0 1 Shininess 0 45")
        modelVisu.addObject('BarycentricMapping')

        rootNode.addObject(Controller(name="EqualityController", RootNode=rootNode))

        return rootNode
