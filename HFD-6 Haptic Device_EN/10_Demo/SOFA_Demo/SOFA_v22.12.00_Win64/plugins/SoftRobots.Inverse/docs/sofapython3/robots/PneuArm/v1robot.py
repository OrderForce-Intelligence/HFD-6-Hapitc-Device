
import Sofa

import os
path = os.path.dirname(os.path.abspath(__file__))+'/robot_challenge/'
meshRobot=path+'springy_robot_tetra_1000vert.vtu'

meshIntervertebra=path+'rigid-intervertebra-oneside.stl'
meshSelect=path+'BiggerSelect.stl'

print("")
print('033[36m' + '033[1m' + " [Informations about the scene]" + '033[0m')
print(" (please refer to the README file for more informations)")
print("")

# utility methods

def fillIndexPairs(indexIn, numPoints):
    max = len(indexIn)

    j=0;
    k=0;
    l=0;
    indexPairs=[1,1]*(numPoints)
    for i in range(numPoints):
        # print [i,j]
        if j<max:
            if  indexIn[j][0]==i :
                indexPairs[2*i  ]=1;
                indexPairs[2*i+1]=k;
                j=j+1;
                k=k+1;
            else :
                indexPairs[2*i  ]=0;
                indexPairs[2*i+1]=l;
                l=l+1;
        else :
            indexPairs[2*i  ]=0;
            indexPairs[2*i+1]=l;
            l=l+1;

    return indexPairs




def createScene(rootNode):
    # Root node
    rootNode.findData('dt').value=0.001;
    rootNode.findData('gravity').value='0 -9.810 0';
    rootNode.addObject('VisualStyle', displayFlags='showVisualModels showForceFields showInteractionForceFields showCollisionModels hideBoundingCollisionModels hideWireframe');

    #Required plugin
    rootNode.addObject('RequiredPlugin', pluginName='SoftRobots');

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver', printLog='0', epsilon='1')
    #rootNode.addObject('GenericConstraintSolver', printLog='1')

    rootNode.addObject('DefaultPipeline', verbose='0')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('DefaultContactManager', response='FrictionContactConstraint')
    rootNode.addObject('LocalMinDistance', name="Proximity", alarmDistance='0', contactDistance='0')


    #rootNode.addObject('ClipPlane', normal='0 0 1')

    #goal
    goal = rootNode.addChild('goal')
    goal.findData('activated').value='1'
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations='100', tolerance="1e-5", threshold="1e-10")
    goal.addObject('MechanicalObject', name='goalMORigide', template="Rigid3", position='0 5.5 0 0 0 0 1')
    goal.addObject('UniformMass',  totalMass='0.1')
    goal.addObject('UncoupledConstraintCorrection')
    goalPoints= goal.addChild('goalPoints')
    goalPoints.addObject('MechanicalObject', name='goalMO', template='Vec3', position='0 0 0 ')
    # 1 0 0  0 1 0  0 0 1
    goalPoints.addObject('SphereCollisionModel', radius='0.2', group='1')
    goalPoints.addObject('RigidMapping', name="rigidMap", input='@..',output='@.')



    #mergeMesh
    mergeMesh = rootNode.addChild('mergeMesh')
    mergeMesh.addObject('MeshVTKLoader', name="mesh1", filename=meshRobot, translation="1.5 0 2.5")
    mergeMesh.addObject('MeshVTKLoader', name="mesh2", filename=meshRobot, translation="1.5 0 -2.5", rotation="0 120 0")
    mergeMesh.addObject('MeshVTKLoader', name="mesh3", filename=meshRobot, translation="-3 0 0", rotation="0 240 0")
    mergeEngine=mergeMesh.addObject('MergeMeshes', name="Merge", nbMeshes="3", position1="@mesh1.position", tetrahedra1="@mesh1.tetrahedra",
                                position2="@mesh2.position", tetrahedra2="@mesh2.tetrahedra",
                                position3="@mesh3.position", tetrahedra3="@mesh3.tetrahedra")


    boxFix= mergeMesh.addObject('BoxROI',template="Vec3", name='box_roi_fix',box='-6 -0.01 -6 6 1 6',drawBoxes='1', drawPoints='0', position="@Merge.position")
    boxIndep=mergeMesh.addObject('BoxROI',template="Vec3", name='box_roi_indep',box='-6 -0.01 -6 6 4 6',drawBoxes='1', drawPoints='0', position="@Merge.position")
    boxFrame1= mergeMesh.addObject('BoxROI',template="Vec3", name='box_roi_frame1',box='-6 4 -6 6 5 6',drawBoxes='1', drawPoints='0', position="@Merge.position")

    # tetra topology => triangle topology
    mergeMesh.addObject('TetrahedronSetTopologyContainer', src='@Merge', name='container')
    mergeMesh.addObject('TetrahedronSetTopologyModifier')

    mergeMesh.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale='4e-5', rx='0', dz='0')
    #mergeMesh.addObject('TetrahedronFEMForceField', name="tetraFF", youngModulus="600", poissonRatio="0.45")
    # triangle topology
    mergeMeshTri = mergeMesh.addChild('tri')
    mergeMeshTri.addObject('TriangleSetTopologyContainer', name='container', position='@../container.position')
    mergeMeshTri.addObject('TriangleSetTopologyModifier')
    mergeMeshTri.addObject('Tetra2TriangleTopologicalMapping', name='Mapping', object1='../container', object2='container')
    mergeMeshTri.addObject('MeshSTLLoader', name="loadCavity11", filename=meshSelect, translation="1.5 0 2.5") #scale3d='1.1 1.1 1.1'
    cavityBox11= mergeMeshTri.addObject('MeshROI', name="meshROI11", position="@../container.position", triangles="@container.triangles", ROIposition="@loadCavity11.position", ROItriangles="@loadCavity11.triangles",computeEdges='0', computeTriangles='1', computeTetrahedra='0' )

    mergeMeshTri.addObject('MeshSTLLoader', name="loadCavity12", filename=meshSelect, translation="1.5 0 -2.5") #scale3d='1.1 1.1 1.1'
    cavityBox12= mergeMeshTri.addObject('MeshROI', name="meshROI12", position="@../container.position", triangles="@container.triangles", ROIposition="@loadCavity12.position", ROItriangles="@loadCavity12.triangles",computeEdges='0', computeTriangles='1', computeTetrahedra='0' )

    mergeMeshTri.addObject('MeshSTLLoader', name="loadCavity13", filename=meshSelect, translation="-3 0 0") #scale3d='1.1 1.1 1.1'
    cavityBox13= mergeMeshTri.addObject('MeshROI', name="meshROI13", position="@../container.position", triangles="@container.triangles", ROIposition="@loadCavity13.position", ROItriangles="@loadCavity13.triangles",computeEdges='0', computeTriangles='1', computeTetrahedra='0' )


    mergeEngine.init();
    numPoints = mergeEngine.findData('npoints').value;
    boxFrame1.init();
    IndicesFrame1= boxFrame1.findData('indices').value;
    #print "PointRigi";
    #print IndicesFrame1;
    print(numPoints)

    print('+++++++++++++++++++++++++++++++++++')
    print(len(IndicesFrame1))
    print('+++++++++++++++++++++++++++++++++++')

    indexPairs = fillIndexPairs(IndicesFrame1, numPoints);
    indexPairsStr = str(indexPairs)


    # composite model
    compositeNode = rootNode.addChild('CompositeModel')
    compositeNode.addObject('EulerImplicitSolver',name='cg_odesolver',printLog=False, rayleighStiffness='1', rayleighMass='0', firstOrder=False)
    compositeNode.addObject('SparseLDLSolver', name="ldlsolveur")

    #compositeNode.addObject('CGLinearSolver', iterations='50', tolerance="1e-5", threshold="1e-10")
    compositeNode.addObject('GenericConstraintCorrection', solverName='ldlsolveur')
    compositeNode.addObject('PythonScriptController', filename="controller2.py", classname="controller")



    # ---------------------------------------------------------------------------------------------
    # ---------------------- Intervertebre rigide ----------------------------------
    # ---------------------------------------------------------------------------------------------
    interv1Node = compositeNode.addChild('Intervertebra1')
    interv1Node.addObject('MechanicalObject', template="Rigid3",name='interv1MO',position='0 5 0 0 0 0 1')
    #top_disc1Node.addObject('UniformMass',name='mass',totalMass='0.000001',showAxisSizeFactor='0.005')
    #interv1Node.addObject('PartialRigidificationConstraint')

    # ---------------------------------------------------------------------------------------------
    # ---------------------- Particules independantes ----------------------------------
    # ---------------------------------------------------------------------------------------------
    indep_particulesNode = compositeNode.addChild('Independant_Particules')
    indep_particulesNode.addObject('PointSetTopologyContainer', position='@../../mergeMesh/box_roi_indep.pointsInROI')
    IndependantParticles_dof=indep_particulesNode.addObject('MechanicalObject', template='Vec3', name='IndependantParticles_dof')
    indep_particulesNode.addObject('UniformMass',name='mass',totalMass='0.1')
    indep_particulesNode.addObject('BoxROI',name='box_roi',box='-6 -0.01 -6 6 1 6',drawBoxes='1', drawPoints='1')
    indep_particulesNode.addObject('RestShapeSpringsForceField', points='@box_roi.indices', stiffness='1e12')
    #indep_particulesNode.addObject('FixedConstraint', indices='@box_roi.indices')

    DeformableGridNode = indep_particulesNode.addChild('DeformableGrid')



    # ---------------------------------------------------------------------------------------------
    # ---------------------- Particules liees au rigide ----------------------
    # ---------------------------------------------------------------------------------------------
    Mapped_ParticuleNode = interv1Node.addChild('Mapped_Particule')
    Mapped_ParticuleNode.addObject('PointSetTopologyContainer', position='@../../../mergeMesh/box_roi_frame1.pointsInROI')
    Mapped_Particles_dof=Mapped_ParticuleNode.addObject('MechanicalObject', template='Vec3', name='Mapped_Particles_dof',  showIndices=False)
    Mapped_ParticuleNode.addObject('RigidMapping', name="rigidMap", input='@..',output='@.', globalToLocalCoords=True)


    # ---------------------------------------------------------------------------------------------
    # ---------------------- Somme des particules et du rigide ----------------------
    # ---------------------------------------------------------------------------------------------

    Mapped_ParticuleNode.addChild(DeformableGridNode)
    DeformableGridNode.addObject('MeshTopology', src='@../../../../mergeMesh/Merge', name="mesh")
    DeformableGridNode.addObject('MechanicalObject',template='Vec3',name='mergeDofs')

    DeformableGridNode.addObject('TetrahedronFEMForceField', name="tetraFF", youngModulus="18000", poissonRatio="0.3")

    deformableGrid_mappaddPointing = DeformableGridNode.addObject('SubsetMultiMapping',template='Vec3,Vec3d',name='deformableGrid_mapping', input='@../../../CompositeModel/Independant_Particules/IndependantParticles_dof @../../../CompositeModel/Intervertebra1/Mapped_Particule/Mapped_Particles_dof ', output='@./mergeDofs', indexPairs=indexPairsStr)
    compositeNode.addObject('PartialRigidificationForceField', object1='@./Independant_Particules/IndependantParticles_dof', object2='@./Intervertebra1/interv1MO', rigidMapping='@./Intervertebra1/Mapped_Particule/rigidMap', subsetMultiMapping='@./Independant_Particules/DeformableGrid/deformableGrid_mapping', mappedForceField='@./Independant_Particules/DeformableGrid/tetraFF' )

    # ---------------------------------------------------------------------------------------------
    # ---------------------- Contraintes (cavity) ----------------------
    # ---------------------------------------------------------------------------------------------
    Cavity11Node=DeformableGridNode.addChild('Cavity11')
    Cavity11Node.findData('activated').value='1'
    Cavity11Node.addObject('MeshTopology', position='@../mesh.position', triangles="@../../../../mergeMesh/tri/meshROI11.trianglesInROI", name="cavityMesh")
    Cavity11Node.addObject('SurfacePressureActuator', template='Vec3', triangles='@cavityMesh.triangles', maxPressure='10', minPressure='0', visualization='1', showVisuScale='0.0002')
    #Cavity11Node.addObject('SurfacePressureConstraint', template='Vec3', triangles='@cavityMesh.triangles', constantPressureValue='1000', visualization='1', showVisuScale='0.0002')
    Cavity11Node.addObject('Triangle')


    Cavity12Node=DeformableGridNode.addChild('Cavity12')
    Cavity12Node.findData('activated').value='1'
    Cavity12Node.addObject('MeshTopology', position='@../mesh.position', triangles="@../../../../mergeMesh/tri/meshROI12.trianglesInROI", name="cavityMesh")
    Cavity12Node.addObject('SurfacePressureActuator', template='Vec3', triangles='@cavityMesh.triangles', maxPressure='10', minPressure='0', visualization='1', showVisuScale='0.0002')
    #Cavity12Node.addObject('SurfacePressureConstraint', template='Vec3', triangles='@cavityMesh.triangles', constantPressureValue='1000', visualization='1', showVisuScale='0.0002')
    Cavity12Node.addObject('Triangle')

    Cavity13Node=DeformableGridNode.addChild('Cavity13')
    Cavity13Node.findData('activated').value='1'
    Cavity13Node.addObject('MeshTopology', position='@../mesh.position', triangles="@../../../../mergeMesh/tri/meshROI13.trianglesInROI", name="cavityMesh")
    Cavity13Node.addObject('SurfacePressureActuator', template='Vec3', triangles='@cavityMesh.triangles', maxPressure='10', minPressure='0', visualization='1', showVisuScale='0.0002')
    #Cavity13Node.addObject('SurfacePressureConstraint', template='Vec3', triangles='@cavityMesh.triangles', constantPressureValue='1000', visualization='1', showVisuScale='0.0002')
    Cavity13Node.addObject('Triangle')


    # ---------------------------------------------------------------------------------------------
    # ---------------------- Contraintes (cables) ----------------------
    # ---------------------------------------------------------------------------------------------
    cables = interv1Node.addChild('cables')
    cables.findData('activated').value='1'
    cables.addObject('MechanicalObject', name="cablePoint", position="3 0 0  -1.5 0 2.5  -1.5 0 -2.5")
    cables.addObject('RigidMapping', name="rigidMapCablePoints", input='@..',output='@.')

    cables.addObject('CableActuator', template="Vec3", name="c1" , indices="0", pullPoint="3 0 0" , maxPositiveDisp="10", maxNegativeDisp="0")
    cables.addObject('CableActuator', template="Vec3", name="c2" , indices="1", pullPoint="-1.5 0 2.5" , maxPositiveDisp="10", maxNegativeDisp="0")
    cables.addObject('CableActuator', template="Vec3", name="c3" , indices="2", pullPoint="-1.5 0 -2.5" , maxPositiveDisp="10", maxNegativeDisp="0")

    #cables.addObject('CableConstraint', template="Vec3", name="c1" , indices="0", pullPoint="3 0 0" , inputValue="0")
    #cables.addObject('CableConstraint', template="Vec3", name="c2" , indices="1", pullPoint="-1.5 0 2.5" , inputValue="0")
    #cables.addObject('CableConstraint', template="Vec3", name="c3" , indices="2", pullPoint="-1.5 0 -2.5" , inputValue="0")



    # ---------------------------------------------------------------------------------------------
    # ---------------------- Contraintes (Effector) ----------------------
    # ---------------------------------------------------------------------------------------------

    # ---------------------- version 6D ----------------------
    #interv1Node.addObject('PositionEffector', template="Rigid3", indices="0", effectorGoal="@../../goal/goalMORigide.position")

    # ---------------------- version mappee ------------------
    effector = interv1Node.addChild('effector')
    effector.addObject('MechanicalObject', name="effectorPoint", position="0 0 0")
    effector.addObject('SphereCollisionModel', radius='0.05', group='1')
    effector.addObject('PositionEffector', template='Vec3', indices="0", effectorGoal="@../../../goal/goalPoints/goalMO.position")
    effector.addObject('RigidMapping', name="rigidMap", input='@..',output='@.')


    #mergeMesh.addObject('MeshVTKLoader', name="mesh2", filename=meshRobot, translation="1.5 0 -2.5")
    #mergeMesh.addObject('MeshVTKLoader', name="mesh3", filename=meshRobot, translation="-3 0 0")


    # ---------------------------------------------------------------------------------------------
    # ---------------------- Visu (Intervertebra) ----------------------
    # ---------------------------------------------------------------------------------------------
    visuIntervertebra= interv1Node.addChild('visuIntervertebra')
    visuIntervertebra.addObject('MeshSTLLoader', name="loadIntervertebra", filename=meshIntervertebra, translation="-3 0 0",  rotation="0 0 0")
    visuIntervertebra.addObject('MeshTopology', src='@loadIntervertebra', name="intervertebraMesh")
    visuIntervertebra.addObject('OglModel')
    visuIntervertebra.addObject('RigidMapping', name="rigidMapVisu", input='@..',output='@.')



    return rootNode
