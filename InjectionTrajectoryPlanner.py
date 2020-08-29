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
        self.needle_filename = self.dir + '/Resources/meshes/50mm_18ga_needle.stl'
        self.test_volume_data_filename = self.dir + '/Resources/volumes/test_spine_segmentation.nrrd'
        self.test_ct_directory = self.dir + '/Resources/images/Case1 CT'  # input folder with DICOM files

        needle_transform_name = 'needle'
        needleMeshModel = SlicerMeshModel(needle_transform_name, self.needle_filename)

        self.targetMarkupNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLMarkupsFiducialNode')
        self.targetMarkupNode.SetName('Target')
        n = slicer.modules.markups.logic().AddFiducial()
        self.targetMarkupNode.SetNthFiducialLabel(n, "Target")

        # each markup is given a unique id which can be accessed from the superclass level
        self.targetFiducialID = self.targetMarkupNode.GetNthMarkupID(n)

        self.referenceMarkupNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLMarkupsFiducialNode')
        self.referenceMarkupNode.SetName('Reference')
        n = slicer.modules.markups.logic().AddFiducial()
        self.referenceMarkupNode.SetNthFiducialLabel(n, "Reference")
        # each markup is given a unique id which can be accessed from the superclass level
        self.referenceFiducialID = self.referenceMarkupNode.GetNthMarkupID(n)

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
        self.beginSelector.setToolTip("Pick the trajectory reference marker")
        self.beginSelector.setCurrentNode(self.referenceMarkupNode)

        parametersFormLayout.addRow("Path Reference Marker: ", self.beginSelector)

        self.rulerNode = slicer.vtkMRMLAnnotationRulerNode()
        self.rulerNode.SetPosition1(-10, -10, -10)
        self.rulerNode.SetPosition2(10, 10, 10)
        self.rulerNode.SetLocked(1)
        self.rulerNode.Initialize(slicer.mrmlScene)
        self.rulerNode.SetTextScale(0)

        self.targetMarkupNode.AddObserver(slicer.vtkMRMLMarkupsNode.PointModifiedEvent,
                                          self.TargetMarkupModifiedCallback)
        self.referenceMarkupNode.AddObserver(slicer.vtkMRMLMarkupsNode.PointModifiedEvent,
                                             self.ReferenceMarkupModifiedCallback)

        self.targetMarkupNode.SetNthFiducialPosition(0, 0, 0, 0)
        self.referenceMarkupNode.SetNthFiducialPosition(0, 100, 100, 100)

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

        self.moveTargetToIntersectionButton = qt.QPushButton("Move Target to Slice Intersection")
        self.moveTargetToIntersectionButton.toolTip = "Align slice intersections (hover while pressing shift may " \
                                                      "help). Click button to move target point here"
        self.moveTargetToIntersectionButton.enabled = True
        parametersFormLayout.addRow(self.moveTargetToIntersectionButton)

        self.moveReferenceToIntersectionButton = qt.QPushButton("Move Reference to Slice Intersection")
        self.moveReferenceToIntersectionButton.toolTip = "Align slice intersections (hover while pressing shift may " \
                                                      "help). Click button to move target point here"
        self.moveReferenceToIntersectionButton.enabled = True
        parametersFormLayout.addRow(self.moveReferenceToIntersectionButton)

        self.jumpToTargetButton = qt.QPushButton("Change Slice View to Target Point")
        self.jumpToTargetButton.toolTip = "Press to see the target point in all slices"
        parametersFormLayout.addRow(self.jumpToTargetButton)

        self.jumpToReferenceButton = qt.QPushButton("Change Slice View to Reference Point")
        self.jumpToReferenceButton.toolTip = "Press to see the reference point in all slices"
        parametersFormLayout.addRow(self.jumpToReferenceButton)



        # connections
        self.addTestDataButton.connect('clicked(bool)', self.onAddTestDataButton)
        self.toggleSliceIntersectionButton.connect('clicked(bool)', self.onToggleSliceIntersectionButton)
        self.toggleSliceVisibilityButton.connect('clicked(bool)', self.onToggleSliceVisibilityButton)

        self.moveTargetToIntersectionButton.connect('clicked(bool)', self.onMoveTargetToIntersectionButton)
        self.moveReferenceToIntersectionButton.connect('clicked(bool)', self.onMoveReferenceToIntersectionButton)

        self.jumpToTargetButton.connect('clicked(bool)', self.onJumpToTargetButton)
        self.jumpToReferenceButton.connect('clicked(bool)', self.onJumpToReferenceButton)


        # Add vertical spacer
        self.layout.addStretch(1)

        # Refresh Apply button state
        # self.onSelect()
        viewNodes = slicer.util.getNodesByClass('vtkMRMLSliceCompositeNode')   # Default is ON
        for viewNode in viewNodes:
          viewNode.SetSliceIntersectionVisibility(1)


    def cleanup(self):
        pass

    def onAddTestDataButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.addTestData(self.test_volume_data_filename, self.test_ct_directory)

    def onToggleSliceIntersectionButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.toggleSliceIntersection()

    def onToggleSliceVisibilityButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.toggleSliceVisibility()

    def onMoveTargetToIntersectionButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.moveTargetToIntersectionButton(self.targetMarkupNode)

    def onMoveReferenceToIntersectionButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.moveReferenceToIntersectionButton(self.referenceMarkupNode)

    def onJumpToTargetButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.jumpToMarkup(self.targetMarkupNode)

    def onJumpToReferenceButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.jumpToMarkup(self.referenceMarkupNode)

    # noinspection PyUnusedLocal
    def TargetMarkupModifiedCallback(self, caller, event):
        pos = [0, 0, 0]
        self.targetMarkupNode.GetNthFiducialPosition(0, pos)
        self.rulerNode.SetPosition1(pos)
        # self.targetTMarkupNode.SetNthFiducialPosition(0,
        self.rulerNode.SetTextScale(0)

    # noinspection PyUnusedLocal
    def ReferenceMarkupModifiedCallback(self, caller, event):
        pos = [0, 0, 0]
        self.referenceMarkupNode.GetNthFiducialPosition(0, pos)
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


    def moveReferenceToIntersectionButton(self, referenceMarkupNode):
        # layoutManager = slicer.app.layoutManager()
        crosshairNode = slicer.util.getNode('Crosshair')
        crosshairPos = crosshairNode.GetCrosshairRAS()
        referenceMarkupNode.SetNthFiducialPosition(0, crosshairPos[0], crosshairPos[1], crosshairPos[2])

    def jumpToMarkup(self, MarkupNode):
        pos = [0, 0, 0]
        MarkupNode.GetNthFiducialPosition(0, pos)

        for name in ['Red','Yellow','Green']:
            sliceNode = slicer.app.layoutManager().sliceWidget(name).mrmlSliceNode()
            sliceNode.JumpSlice(pos[0], pos[1], pos[2])

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
