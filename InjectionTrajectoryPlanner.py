import os
import unittest
import vtk
import qt
import ctk
import slicer
from slicer.ScriptedLoadableModule import *
import logging
import os


#
# InjectionTrajectoryPlanner
#


class InjectionTrajectoryPlanner(ScriptedLoadableModule):
    """Uses ScriptedLoadableModule base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = "InjectionTrajectoryPlanner"  # TODO make this more human readable by adding spaces
        self.parent.categories = ["SpineRobot"]
        self.parent.dependencies = []
        self.parent.contributors = [
            "Henry Phalen (Johns Hopkins University)"]  # replace with "Firstname Lastname (Organization)"
        self.parent.helpText = """
This module can be used to plan injection trajectories.
"""
        self.parent.helpText += self.getDefaultModuleDocumentationLink()
        self.parent.acknowledgementText = """
This file was originally developed by Henry Phalen, a PhD student at Johns Hopkins University.
"""  # replace with organization, grant and thanks.


def setSlicePoseFromSliceNormalAndPosition(sliceNode, sliceNormal, slicePosition, defaultViewUpDirection=None,
                                           backupViewRightDirection=None):
    """
    Set slice pose from the provided plane normal and position. View up direction is determined automatically,
    to make view up point towards defaultViewUpDirection.
    :param sliceNode:
    :param sliceNormal:
    :param slicePosition:
    :param defaultViewUpDirection: Slice view will be spinned in-plane to match point approximately this up direction.
    Default: patient superior.
    :param backupViewRightDirection Slice view will be spinned in-plane to match point approximately this right
    direction if defaultViewUpDirection is too similar to sliceNormal. Default: patient left.
    This is from:
    https://www.slicer.org/wiki/Documentation/Nightly/ScriptRepository#Set_slice_position_and_orientation_from_a_normal_vector_and_position
    """
    # Fix up input directions
    if defaultViewUpDirection is None:
        defaultViewUpDirection = [0, 0, 1]
    if backupViewRightDirection is None:
        backupViewRightDirection = [-1, 0, 0]
    if sliceNormal[1] >= 0:
        sliceNormalStandardized = sliceNormal
    else:
        sliceNormalStandardized = [-sliceNormal[0], -sliceNormal[1], -sliceNormal[2]]
    # Compute slice axes
    sliceNormalViewUpAngle = vtk.vtkMath.AngleBetweenVectors(sliceNormalStandardized, defaultViewUpDirection)
    angleTooSmallThresholdRad = 0.25  # about 15 degrees
    if angleTooSmallThresholdRad < sliceNormalViewUpAngle < vtk.vtkMath.Pi() - angleTooSmallThresholdRad:
        viewUpDirection = defaultViewUpDirection
        sliceAxisY = viewUpDirection
        sliceAxisX = [0, 0, 0]
        vtk.vtkMath.Cross(sliceAxisY, sliceNormalStandardized, sliceAxisX)
    else:
        sliceAxisX = backupViewRightDirection
        # Set slice axes
        sliceNode.SetSliceToRASByNTP(sliceNormalStandardized[0], sliceNormalStandardized[1], sliceNormalStandardized[2],
                                     sliceAxisX[0], sliceAxisX[1], sliceAxisX[2],
                                     slicePosition[0], slicePosition[1], slicePosition[2], 0)


#
# InjectionTrajectoryPlannerWidget
#


class SlicerMeshModel:
    """ Sets up transforms in Slicer and retains their information """

    def __init__(self, transform_name, mesh_filename):
        self.transform_name = transform_name
        self.mesh_filename = mesh_filename
        _, self.mesh_model_node = slicer.util.loadModel(mesh_filename, returnNode=True)
        self.mesh_nodeID = self.mesh_model_node.GetID()
        self.transform_node = slicer.vtkMRMLTransformNode()
        self.transform_node.SetName(transform_name)
        slicer.mrmlScene.AddNode(self.transform_node)
        self.transform_nodeID = self.transform_node.GetID()
        self.mesh_model_node.SetAndObserveTransformNodeID(self.transform_nodeID)


# noinspection PyAttributeOutsideInit,PyMethodMayBeStatic
class InjectionTrajectoryPlannerWidget(ScriptedLoadableModuleWidget):
    """Uses ScriptedLoadableModuleWidget base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """

    def setup(self):
        ScriptedLoadableModuleWidget.setup(self)

        # Instantiate and connect widgets ...

        #
        # Parameters Area
        #
        parametersCollapsibleButton = ctk.ctkCollapsibleButton()
        parametersCollapsibleButton.text = "Parameters"
        self.layout.addWidget(parametersCollapsibleButton)

        # Layout within the dummy collapsible button
        parametersFormLayout = qt.QFormLayout(parametersCollapsibleButton)

        #
        # input volume selector
        #
        import os
        self.dir = os.path.dirname(__file__)
        # self.needle_filename = self.dir + '/Resources/meshes/50mm_18ga_needle.stl'
        self.test_volume_data_filename = self.dir + '/Resources/volumes/test_spine_segmentation.nrrd'
        self.test_ct_directory = self.dir + '/Resources/images/Case1 CT'  # input folder with DICOM files

        # needle_transform_name = 'needle'
        # needleMeshModel = SlicerMeshModel(needle_transform_name, self.needle_filename)

        self.targetMarkupNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLMarkupsFiducialNode')
        self.targetMarkupNode.SetName('Target')
        n = slicer.modules.markups.logic().AddFiducial()
        self.targetMarkupNode.SetNthFiducialLabel(n, "Target")

        # each markup is given a unique id which can be accessed from the superclass level
        self.targetFiducialID = self.targetMarkupNode.GetNthMarkupID(n)

        self.EntryMarkupNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLMarkupsFiducialNode')
        self.EntryMarkupNode.SetName('Entry')
        n = slicer.modules.markups.logic().AddFiducial()
        self.EntryMarkupNode.SetNthFiducialLabel(n, "Entry")
        # each markup is given a unique id which can be accessed from the superclass level
        self.EntryFiducialID = self.EntryMarkupNode.GetNthMarkupID(n)

        self.targetSelector = slicer.qMRMLNodeComboBox()
        self.targetSelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
        self.targetSelector.selectNodeUponCreation = True
        self.targetSelector.addEnabled = True
        self.targetSelector.removeEnabled = False
        self.targetSelector.noneEnabled = False
        self.targetSelector.showHidden = False
        self.targetSelector.showChildNodeTypes = False
        self.targetSelector.setMRMLScene(slicer.mrmlScene)
        self.targetSelector.setToolTip("Pick the target marker")
        self.targetSelector.setCurrentNode(self.targetMarkupNode)
        parametersFormLayout.addRow("Target Marker: ", self.targetSelector)

        #
        # output volume selector
        #
        self.beginSelector = slicer.qMRMLNodeComboBox()
        self.beginSelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
        self.beginSelector.selectNodeUponCreation = True
        self.beginSelector.addEnabled = True
        self.beginSelector.removeEnabled = True
        self.beginSelector.noneEnabled = False
        self.beginSelector.showHidden = False
        self.beginSelector.showChildNodeTypes = False
        self.beginSelector.setMRMLScene(slicer.mrmlScene)
        self.beginSelector.setToolTip("Pick the trajectory Entry marker")
        self.beginSelector.setCurrentNode(self.EntryMarkupNode)

        parametersFormLayout.addRow("Path Entry Marker: ", self.beginSelector)

        self.rulerNode = slicer.vtkMRMLAnnotationRulerNode()
        self.rulerNode.SetPosition1(-10, -10, -10)
        self.rulerNode.SetPosition2(10, 10, 10)
        self.rulerNode.SetLocked(1)
        self.rulerNode.Initialize(slicer.mrmlScene)
        self.rulerNode.SetTextScale(0)
        self.rulerNode.SetName('Trajectory')

        self.rulerTextNode = self.rulerNode.GetAnnotationTextDisplayNode()
        self.rulerTextNode.SetVisibility(0)
        self.rulerLineNode = self.rulerNode.GetAnnotationLineDisplayNode()
        self.rulerLineNode.SetMaxTicks(0)
        self.rulerLineNode.SetSliceProjection(1)
        self.rulerLineNode.SetProjectedColor(0, 1, 1)  # cyan

        self.targetMarkupNode.AddObserver(slicer.vtkMRMLMarkupsNode.PointModifiedEvent,
                                          self.TargetMarkupModifiedCallback)
        self.EntryMarkupNode.AddObserver(slicer.vtkMRMLMarkupsNode.PointModifiedEvent,
                                         self.EntryMarkupModifiedCallback)

        self.targetMarkupNode.SetNthFiducialPosition(0, 0, 0, 0)
        self.EntryMarkupNode.SetNthFiducialPosition(0, 100, 100, 100)

        # Add test volume/image data
        self.addTestDataButton = qt.QPushButton("Add Test Data")
        self.addTestDataButton.toolTip = "Add test volume/image data"
        self.addTestDataButton.enabled = True
        parametersFormLayout.addRow(self.addTestDataButton)

        self.toggleSliceIntersectionButton = qt.QPushButton("Toggle Slice Intersections")
        self.toggleSliceIntersectionButton.toolTip = "Turn on / off colored lines representing slice planes"
        self.toggleSliceIntersectionButton.enabled = True
        parametersFormLayout.addRow(self.toggleSliceIntersectionButton)

        self.toggleSliceVisibilityButton = qt.QPushButton("Toggle Slice Visualization in Rendering")
        self.toggleSliceVisibilityButton.toolTip = "Turn on/off Slice Visualization in 3D Rendering"
        self.toggleSliceVisibilityButton.enabled = True
        parametersFormLayout.addRow(self.toggleSliceVisibilityButton)

        self.moveTargetToIntersectionButton = qt.QPushButton("SET Target Point to Slice Intersection")
        self.moveTargetToIntersectionButton.toolTip = "Align slice intersections (hover while pressing shift may " \
                                                      "help). Click button to move target point here"
        self.moveTargetToIntersectionButton.enabled = True
        parametersFormLayout.addRow(self.moveTargetToIntersectionButton)

        self.moveEntryToIntersectionButton = qt.QPushButton("SET Entry Point to Slice Intersection")
        self.moveEntryToIntersectionButton.toolTip = "Align slice intersections (hover while pressing shift may " \
                                                     "help). Click button to move target point here"
        self.moveEntryToIntersectionButton.enabled = True
        parametersFormLayout.addRow(self.moveEntryToIntersectionButton)

        self.jumpToTargetButton = qt.QPushButton("View Target Point")
        self.jumpToTargetButton.toolTip = "Press to see the target point in all slices"
        parametersFormLayout.addRow(self.jumpToTargetButton)

        self.jumpToEntryButton = qt.QPushButton("View Entry Point")
        self.jumpToEntryButton.toolTip = "Press to see the Entry point in all slices"
        parametersFormLayout.addRow(self.jumpToEntryButton)

        self.alignAxesToTrajectoryButton = qt.QPushButton("Change Slice Views to look down trajectory")
        self.alignAxesToTrajectoryButton.toolTip = "Axial view switches to down-trajectory view. " \
                                                   "Other planes rotate by same amount to remain orthogonal"
        parametersFormLayout.addRow(self.alignAxesToTrajectoryButton)

        self.alignAxesToASCButton = qt.QPushButton("Change Slice Views to standard A-S-C")
        self.alignAxesToASCButton.toolTip = "Returns to default axial, sagittal, coronal slice views"
        parametersFormLayout.addRow(self.alignAxesToASCButton)

        # connections
        self.addTestDataButton.connect('clicked(bool)', self.onAddTestDataButton)
        self.toggleSliceIntersectionButton.connect('clicked(bool)', self.onToggleSliceIntersectionButton)
        self.toggleSliceVisibilityButton.connect('clicked(bool)', self.onToggleSliceVisibilityButton)

        self.moveTargetToIntersectionButton.connect('clicked(bool)', self.onMoveTargetToIntersectionButton)
        self.moveEntryToIntersectionButton.connect('clicked(bool)', self.onMoveEntryToIntersectionButton)

        self.jumpToTargetButton.connect('clicked(bool)', self.onJumpToTargetButton)
        self.jumpToEntryButton.connect('clicked(bool)', self.onJumpToEntryButton)

        self.alignAxesToTrajectoryButton.connect('clicked(bool)', self.onAlignAxesToTrajectoryButton)
        self.alignAxesToASCButton.connect('clicked(bool)', self.onAlignAxesToASCButton)

        # Add vertical spacer
        self.layout.addStretch(1)

        # Refresh Apply button state
        # self.onSelect()
        viewNodes = slicer.util.getNodesByClass('vtkMRMLSliceCompositeNode')  # Default is ON
        for viewNode in viewNodes:
            viewNode.SetSliceIntersectionVisibility(1)

        crosshairNode = slicer.util.getNode('Crosshair')  # Make sure exists
        crosshairPos = crosshairNode.SetCrosshairRAS(0, 0, 0)

    def cleanup(self):
        pass

    def onAddTestDataButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.addTestData(self.test_volume_data_filename, self.test_ct_directory)
        slicer.util.selectModule('InjectionTrajectoryPlanner')  # Switch back after going to DICOM module

    def onToggleSliceIntersectionButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.toggleSliceIntersection()

    def onToggleSliceVisibilityButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.toggleSliceVisibility()

    def onMoveTargetToIntersectionButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.moveTargetToIntersectionButton(self.targetMarkupNode)

    def onMoveEntryToIntersectionButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.moveEntryToIntersectionButton(self.EntryMarkupNode)

    def onJumpToTargetButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.jumpToMarkup(self.targetMarkupNode)

    def onJumpToEntryButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.jumpToMarkup(self.EntryMarkupNode)

    def onAlignAxesToTrajectoryButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.alignAxesWithTrajectory(self.targetMarkupNode, self.EntryMarkupNode)
        self.onJumpToTargetButton()  # return crosshair to target point

        layoutManager = slicer.app.layoutManager()
        threeDWidget = layoutManager.threeDWidget(0)
        threeDView = threeDWidget.threeDView()
        threeDView.resetFocalPoint()

    def onAlignAxesToASCButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.resetAxesToASC(self.targetMarkupNode)
        self.onJumpToTargetButton()  # return crosshair to target point

        layoutManager = slicer.app.layoutManager()
        threeDWidget = layoutManager.threeDWidget(0)
        threeDView = threeDWidget.threeDView()
        threeDView.resetFocalPoint()

    # noinspection PyUnusedLocal
    def TargetMarkupModifiedCallback(self, caller, event):
        pos = [0, 0, 0]
        self.targetMarkupNode.GetNthFiducialPosition(0, pos)
        self.rulerNode.SetPosition1(pos)
        # self.targetTMarkupNode.SetNthFiducialPosition(0,
        self.rulerNode.SetTextScale(0)

    # noinspection PyUnusedLocal
    def EntryMarkupModifiedCallback(self, caller, event):
        pos = [0, 0, 0]
        self.EntryMarkupNode.GetNthFiducialPosition(0, pos)
        self.rulerNode.SetPosition2(pos)
        self.rulerNode.SetTextScale(0)


#
# InjectionTrajectoryPlannerLogic
#

# noinspection PyMethodMayBeStatic
class InjectionTrajectoryPlannerLogic(ScriptedLoadableModuleLogic):
    """This class should implement all the actual
  computation done by your module.  The interface
  should be such that other python code can import
  this class and make use of the functionality without
  requiring an instance of the Widget.
  Uses ScriptedLoadableModuleLogic base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """

    def addTestData(self, test_volume_data_filename, test_ct_directory):
        """ Load volume data  """
        _, label_volumeNode = slicer.util.loadLabelVolume(test_volume_data_filename, returnNode=True)

        # This is adapted from https://www.slicer.org/wiki/Documentation/4.3/Modules/VolumeRendering
        # The effect is that the volume rendering changes when the segmentation array changes
        slicer_logic = slicer.modules.volumerendering.logic()
        displayNode = slicer_logic.CreateVolumeRenderingDisplayNode()
        slicer.mrmlScene.AddNode(displayNode)
        displayNode.UnRegister(slicer_logic)
        slicer_logic.UpdateDisplayNodeFromVolumeNode(displayNode, label_volumeNode)
        label_volumeNode.AddAndObserveDisplayNodeID(displayNode.GetID())

        """ Load image data  """
        from DICOMLib import DICOMUtils
        with DICOMUtils.TemporaryDICOMDatabase() as db:
            DICOMUtils.importDicom(test_ct_directory, db)
            patientUID = db.patients()[0]
            DICOMUtils.loadPatientByUID(patientUID)
        # voxel_array = slicer.util.arrayFromVolume(label_volumeNode)

        # Resets focal point (pink wireframe bounding box) to center of scene
        layoutManager = slicer.app.layoutManager()
        threeDWidget = layoutManager.threeDWidget(0)
        threeDView = threeDWidget.threeDView()
        threeDView.resetFocalPoint()

    def toggleSliceIntersection(self):
        viewNodes = slicer.util.getNodesByClass('vtkMRMLSliceCompositeNode')
        for viewNode in viewNodes:
            viewNode.SetSliceIntersectionVisibility(int(not viewNode.GetSliceIntersectionVisibility()))

    def toggleSliceVisibility(self):
        layoutManager = slicer.app.layoutManager()
        for sliceViewName in layoutManager.sliceViewNames():
            controller = layoutManager.sliceWidget(sliceViewName).sliceController()
            controller.setSliceVisible(int(not controller.sliceLogic().GetSliceNode().GetSliceVisible()))

    def moveTargetToIntersectionButton(self, targetMarkupNode):
        # layoutManager = slicer.app.layoutManager()
        crosshairNode = slicer.util.getNode('Crosshair')
        crosshairPos = crosshairNode.GetCrosshairRAS()
        targetMarkupNode.SetNthFiducialPosition(0, crosshairPos[0], crosshairPos[1], crosshairPos[2])

    def moveEntryToIntersectionButton(self, EntryMarkupNode):
        # layoutManager = slicer.app.layoutManager()
        crosshairNode = slicer.util.getNode('Crosshair')
        crosshairPos = crosshairNode.GetCrosshairRAS()
        EntryMarkupNode.SetNthFiducialPosition(0, crosshairPos[0], crosshairPos[1], crosshairPos[2])

    def jumpToMarkup(self, MarkupNode):
        pos = [0, 0, 0]
        MarkupNode.GetNthFiducialPosition(0, pos)

        for name in ['Red', 'Yellow', 'Green']:
            sliceNode = slicer.app.layoutManager().sliceWidget(name).mrmlSliceNode()
            sliceNode.JumpSlice(pos[0], pos[1], pos[2])

    def alignAxesWithTrajectory(self, targetMarkupNode, EntryMarkupNode):
        import numpy as np

        redSliceNode = slicer.util.getNode('vtkMRMLSliceNodeRed')
        yellowSliceNode = slicer.util.getNode('vtkMRMLSliceNodeYellow')
        greenSliceNode = slicer.util.getNode('vtkMRMLSliceNodeGreen')

        redSliceToRAS = redSliceNode.GetSliceToRAS()
        yellowSliceToRAS = yellowSliceNode.GetSliceToRAS()
        greenSliceToRAS = greenSliceNode.GetSliceToRAS()

        p_target = np.array([0, 0, 0])
        p_Entry = np.array([0, 0, 0])

        targetMarkupNode.GetNthFiducialPosition(0, p_target)
        EntryMarkupNode.GetNthFiducialPosition(0, p_Entry)
        sliceNormal = p_target - p_Entry
        slicePosition = p_target
        setSlicePoseFromSliceNormalAndPosition(redSliceNode, sliceNormal, slicePosition)

        swap_yellow = vtk.vtkMatrix4x4()
        swap_yellow.SetElement(1, 1, 0)
        swap_yellow.SetElement(2, 1, 1)
        swap_yellow.SetElement(1, 2, 1)
        swap_yellow.SetElement(2, 2, 0)
        vtk.vtkMatrix4x4().Multiply4x4(redSliceToRAS, swap_yellow, yellowSliceToRAS)
        yellowSliceToRAS.SetElement(0, 3, p_target[0])
        yellowSliceToRAS.SetElement(1, 3, p_target[1])
        yellowSliceToRAS.SetElement(2, 3, p_target[2])
        yellowSliceNode.UpdateMatrices()

        swap_green = vtk.vtkMatrix4x4()
        swap_green.SetElement(0, 0, 0)
        swap_green.SetElement(1, 0, -1)
        swap_green.SetElement(1, 1, 0)
        swap_green.SetElement(2, 1, 1)
        swap_green.SetElement(0, 2, -1)
        swap_green.SetElement(2, 2, 0)
        vtk.vtkMatrix4x4().Multiply4x4(redSliceToRAS, swap_green, greenSliceToRAS)
        greenSliceToRAS.SetElement(0, 3, p_target[0])
        greenSliceToRAS.SetElement(1, 3, p_target[1])
        greenSliceToRAS.SetElement(2, 3, p_target[2])

        greenSliceNode.UpdateMatrices()

    def resetAxesToASC(self, targetMarkupNode):
        import numpy as np
        redSliceNode = slicer.util.getNode('vtkMRMLSliceNodeRed')
        yellowSliceNode = slicer.util.getNode('vtkMRMLSliceNodeYellow')
        greenSliceNode = slicer.util.getNode('vtkMRMLSliceNodeGreen')
        redSliceNode.SetOrientationToAxial()
        yellowSliceNode.SetOrientationToSagittal()
        greenSliceNode.SetOrientationToCoronal()
        p_target = np.array([0, 0, 0])
        targetMarkupNode.GetNthFiducialPosition(0, p_target)

        redSliceToRAS = redSliceNode.GetSliceToRAS()
        redSliceToRAS.SetElement(0, 3, p_target[0])
        redSliceToRAS.SetElement(1, 3, p_target[1])
        redSliceToRAS.SetElement(2, 3, p_target[2])
        yellowSliceToRAS = yellowSliceNode.GetSliceToRAS()
        yellowSliceToRAS.SetElement(0, 3, p_target[0])
        yellowSliceToRAS.SetElement(1, 3, p_target[1])
        yellowSliceToRAS.SetElement(2, 3, p_target[2])
        greenSliceToRAS = greenSliceNode.GetSliceToRAS()
        greenSliceToRAS.SetElement(0, 3, p_target[0])
        greenSliceToRAS.SetElement(1, 3, p_target[1])
        greenSliceToRAS.SetElement(2, 3, p_target[2])


# noinspection PyMethodMayBeStatic
class InjectionTrajectoryPlannerTest(ScriptedLoadableModuleTest):
    """
  This is the test case for your scripted module.
  Uses ScriptedLoadableModuleTest base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """

    def setUp(self):
        """ Do whatever is needed to reset the state - typically a scene clear will be enough.
    """
        slicer.mrmlScene.Clear(0)

    def runTest(self):
        """Run as few or as many tests as needed here.
    """
        self.setUp()
        self.test_InjectionTrajectoryPlanner1()

    def test_InjectionTrajectoryPlanner1(self):
        """ Ideally you should have several levels of tests.  At the lowest level
    tests should exercise the functionality of the logic with different inputs
    (both valid and invalid).  At higher levels your tests should emulate the
    way the user would interact with your code and confirm that it still works
    the way you intended.
    One of the most important features of the tests is that it should alert other
    developers when their changes will have an impact on the behavior of your
    module.  For example, if a developer removes a feature that you depend on,
    your test should break so they know that the feature is needed.
    """

        self.delayDisplay("Starting the test")

        self.delayDisplay('Test passed!')
