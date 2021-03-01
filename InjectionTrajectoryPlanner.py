import os
import unittest
import vtk
import qt
import ctk
import slicer
from slicer.ScriptedLoadableModule import *
import logging
import os
from Resources.slicer_helper import slicer_helper as sh
import numpy as np
from datetime import datetime
import os


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
        self.logic = InjectionTrajectoryPlannerLogic()
        self.redSliceNode = slicer.util.getNode('vtkMRMLSliceNodeRed')
        self.lastRedSliceOffset = self.redSliceNode.GetSliceOffset()
        self.trajNumMax = 1  # Need to keep track of this so we can name new trajectories correctly
        self.obliqueRotatorLastStaticPos = np.array([0.0, 0.0, 0.0])

        actionsCollapsibleButton = ctk.ctkCollapsibleButton()
        actionsCollapsibleButton.text = "Actions"
        self.layout.addWidget(actionsCollapsibleButton)
        actionsFormLayout = qt.QFormLayout(actionsCollapsibleButton)

        vizCollapsibleButton = ctk.ctkCollapsibleButton()
        vizCollapsibleButton.text = "Visualization"
        self.layout.addWidget(vizCollapsibleButton)
        vizFormLayout = qt.QFormLayout(vizCollapsibleButton)

        slicevizCollapsibleButton = ctk.ctkCollapsibleButton()
        slicevizCollapsibleButton.text = "Toggle Slice Display in 3D View"
        self.layout.addWidget(slicevizCollapsibleButton)
        slicevizFormLayout = qt.QFormLayout(slicevizCollapsibleButton)


        parametersCollapsibleButton = ctk.ctkCollapsibleButton()
        parametersCollapsibleButton.text = "Parameters"
        self.layout.addWidget(parametersCollapsibleButton)
        parametersCollapsibleButton.setChecked(False)  # closes by default
        parametersFormLayout = qt.QFormLayout(parametersCollapsibleButton)



        #
        # input volume selector
        #
        self.dir = os.path.dirname(__file__)
        self.outdir = self.dir+'/Output/'+datetime.now().strftime('%Y%m%d%H%M%S')
        self.needle_filename = self.dir + '/Resources/meshes/50mm_18ga_needle.stl'
        self.test_volume_data_filename = self.dir + '/Resources/volumes/test_spine_segmentation.nrrd'
        self.test_ct_directory = self.dir + '/Resources/images/Case1 CT'  # input folder with DICOM files

        """Target Point Markup & Selector"""
        # self.targetMarkupNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLMarkupsFiducialNode')
        # self.targetMarkupNode.SetName('Target')
        # n = slicer.modules.markups.logic().AddFiducial()
        # self.targetMarkupNode.SetNthFiducialLabel(n, "Target")
        # # each markup is given a unique id which can be accessed from the superclass level
        # self.targetFiducialID = self.targetMarkupNode.GetNthMarkupID(n)

        # self.targetSelector = slicer.qMRMLNodeComboBox()
        # self.targetSelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
        # self.targetSelector.selectNodeUponCreation = True
        # self.targetSelector.addEnabled = True
        # self.targetSelector.removeEnabled = False
        # self.targetSelector.noneEnabled = False
        # self.targetSelector.showHidden = False
        # self.targetSelector.showChildNodeTypes = False
        # self.targetSelector.setMRMLScene(slicer.mrmlScene)
        # self.targetSelector.setToolTip("Pick the target marker")
        # self.targetSelector.setCurrentNode(self.targetMarkupNode)
        # parametersFormLayout.addRow("Target Marker: ", self.targetSelector)

        """Entry Point Markup & Selector"""
        # self.entryMarkupNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLMarkupsFiducialNode')
        # self.entryMarkupNode.SetName('Entry')
        # n = slicer.modules.markups.logic().AddFiducial()
        # self.entryMarkupNode.SetNthFiducialLabel(n, "Entry")
        # # each markup is given a unique id which can be accessed from the superclass level
        # self.EntryFiducialID = self.entryMarkupNode.GetNthMarkupID(n)

        # self.entrySelector = slicer.qMRMLNodeComboBox()
        # self.entrySelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
        # self.entrySelector.selectNodeUponCreation = True
        # self.entrySelector.addEnabled = True
        # self.entrySelector.removeEnabled = True
        # self.entrySelector.noneEnabled = False
        # self.entrySelector.showHidden = False
        # self.entrySelector.showChildNodeTypes = False
        # self.entrySelector.setMRMLScene(slicer.mrmlScene)
        # self.entrySelector.setToolTip("Pick the trajectory Entry marker")
        # self.entrySelector.setCurrentNode(self.entryMarkupNode)
        # parametersFormLayout.addRow("Path Entry Marker: ", self.entrySelector)

        """Tool Model Markup & Selector"""
        self.entry_transform_node = slicer.vtkMRMLTransformNode()
        self.entry_transform_node.SetName('needle_entry')
        slicer.mrmlScene.AddNode(self.entry_transform_node)

        self.hand_eye_node = slicer.vtkMRMLTransformNode()
        self.hand_eye_node.SetName('hand_eye')
        slicer.mrmlScene.AddNode(self.hand_eye_node)

        self.robot_ee_entry = slicer.vtkMRMLTransformNode()
        self.robot_ee_entry.SetName('robot_ee_entry')
        slicer.mrmlScene.AddNode(self.robot_ee_entry)

        self.robot_ee_target = slicer.vtkMRMLTransformNode()
        self.robot_ee_target.SetName('robot_ee_target')
        slicer.mrmlScene.AddNode(self.robot_ee_target)
        #	self.entry_ee = self.entry_transform_node*inv(self.hand_eye)
        #	self.target_ee = self.toolMeshModel.transform_node*inv(self.hand_eye)

        needle_transform_name = 'needle'
        self.toolMeshModel = sh.SlicerMeshModel(needle_transform_name, self.needle_filename)
        print(self.toolMeshModel)
        self.toolMeshModel.display_node.SetSliceIntersectionVisibility(True)
        self.toolMeshModel.display_node.SetSliceDisplayModeToIntersection()
        self.toolMeshModel.display_node.SetColor((255.0/255.0, 170.0/255.0, 0.0))  # Orange
        self.toolMeshModel.mesh_model_node.SetHideFromEditors(1)

        self.toolModelSelector = slicer.qMRMLNodeComboBox()
        self.toolModelSelector.nodeTypes = ["vtkMRMLModelNode"]
        self.toolModelSelector.selectNodeUponCreation = True
        self.toolModelSelector.addEnabled = True
        self.toolModelSelector.removeEnabled = True
        self.toolModelSelector.noneEnabled = False
        self.toolModelSelector.showHidden = False
        self.toolModelSelector.showChildNodeTypes = False
        self.toolModelSelector.setMRMLScene(slicer.mrmlScene)
        self.toolModelSelector.setToolTip("Pick the tool model")
        self.toolModelSelector.setCurrentNode(self.toolMeshModel.mesh_model_node)
        parametersFormLayout.addRow("Tool Model: ", self.toolModelSelector)

        """Trajectory Line"""
        # Call UpdateSphere whenever the fiducials are changed
        self.selectedTraj = sh.SlicerTrajectoryModel(1)
        self.addSelectedTrajObservers(self.selectedTraj)
        self.trajList = np.array([self.selectedTraj])

        # self.targetMarkupNode.AddObserver(slicer.vtkMRMLMarkupsNode.PointModifiedEvent,
        #                                   self.targetMarkupModifiedCallback)
        # self.entryMarkupNode.AddObserver(slicer.vtkMRMLMarkupsNode.PointModifiedEvent,
        #                                  self.entryMarkupModifiedCallback)
        #
        # self.targetMarkupNode.AddObserver(slicer.vtkMRMLMarkupsNode.PointEndInteractionEvent,
        #                                   self.targetMarkupEndInteractionCallback)
        # self.entryMarkupNode.AddObserver(slicer.vtkMRMLMarkupsNode.PointEndInteractionEvent,
        #                                   self.entryMarkupEndInteractionCallback)
        # self.targetMarkupNode.SetNthFiducialPosition(0, 0, 0, 0)
        # self.entryMarkupNode.SetNthFiducialPosition(0, 100, 100, 100)


        self.downAxisBool = False
        self.lastDATFromProjection = False  # TODO: Make this less terrible
        # down-axis trajectory markup node
        self.num_DAT_screens = 1
        self.DATrajectoryMarkupNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLMarkupsFiducialNode')
        self.DATrajectoryMarkupNode.SetName('Trajectory')
        self.DATrajectoryFiducialIDs = []
        for i in range(self.num_DAT_screens):
            slicer.modules.markups.logic().AddFiducial()  # TODO: Multiple screens handled by different 'n's
            self.DATrajectoryMarkupNode.SetNthFiducialLabel(i, "Trajectory")
            # each markup is given a unique id which can be accessed from the superclass level
            self.DATrajectoryFiducialIDs.append(self.DATrajectoryMarkupNode.GetNthMarkupID(i))
            self.DATrajectoryMarkupNode.SetNthFiducialVisibility(i, False)
        # self.DATrajectoryMarkupNode.AddObserver(slicer.vtkMRMLMarkupsNode.PointEndInteractionEvent,
        #                                         self.downAxisPointCallback)
        self.redSliceNode.AddObserver(vtk.vtkCommand.ModifiedEvent, self.redSliceModifiedCallback)
        self.DATrajectoryMarkupNode.AddObserver(slicer.vtkMRMLMarkupsNode.PointEndInteractionEvent,
                                                self.DATPointEndInteractionCallback)


        """Add Test Data Button"""
        self.addTestDataButton = qt.QPushButton("Add Test Data")
        self.addTestDataButton.toolTip = "Add test volume/image data"
        self.addTestDataButton.enabled = True
        parametersFormLayout.addRow(self.addTestDataButton)

        """Add Test Data Button"""
        self.addTrajFromFileButton = qt.QPushButton("Add Trajectories from File")
        self.addTrajFromFileButton.toolTip = "Add previously generated trajectories from an output file"
        self.addTrajFromFileButton.enabled = True
        parametersFormLayout.addRow(self.addTrajFromFileButton)

        """Toggle Slice Intersections Button"""
        self.toggleSliceIntersectionButton = qt.QPushButton("Toggle Slice Intersections")
        self.toggleSliceIntersectionButton.toolTip = "Turn on / off colored lines representing slice planes"
        self.toggleSliceIntersectionButton.enabled = True
        parametersFormLayout.addRow(self.toggleSliceIntersectionButton)
        self.viewNodes = slicer.util.getNodesByClass('vtkMRMLSliceCompositeNode')  # Default is ON
        for viewNode in self.viewNodes:
            viewNode.SetSliceIntersectionVisibility(1)

        """Toggle Slice Visualization Button [TODO:Separate to 3]"""
        self.toggleSliceVisibilityButtonLayout = qt.QHBoxLayout()
        self.toggleRedSliceVisibilityButton = qt.QPushButton("RED")
        self.toggleRedSliceVisibilityButton.toolTip = "Turn on/off Red Slice Visualization in 3D Rendering"
        self.toggleYellowSliceVisibilityButton = qt.QPushButton("YELLOW")
        self.toggleYellowSliceVisibilityButton.toolTip = "Turn on/off Yellow Slice Visualization in 3D Rendering"
        self.toggleGreenSliceVisibilityButton = qt.QPushButton("GREEN")
        self.toggleGreenSliceVisibilityButton.toolTip = "Turn on/off Green Slice Visualization in 3D Rendering"
        self.toggleSliceVisibilityButtonLayout.addWidget(self.toggleRedSliceVisibilityButton)
        self.toggleSliceVisibilityButtonLayout.addWidget(self.toggleYellowSliceVisibilityButton)
        self.toggleSliceVisibilityButtonLayout.addWidget(self.toggleGreenSliceVisibilityButton)
        slicevizFormLayout.addRow(self.toggleSliceVisibilityButtonLayout)

        """Instruction Text"""
        x = qt.QLabel()
        x.setText("Helpful Controls:\n"
                  " - Hold SHIFT while hovering mouse to change slice intersection\n"
                  " - Right click and drag for zoom\n"
                  " - Left click and drag for image settings\n"
                  " - Click and drag points to move (only selected trajectory)\n"
                  " - Mouse scroll between slices\n"
                  " - Hold mouse scroll wheel and drag to pan slices\n")
        x.setWordWrap(True)

        actionsFormLayout.addRow(x)

        """Select Trajectory"""
        # self.trajSelector = slicer.qMRMLNodeComboBox()
        # self.trajSelector.nodeTypes = ["vtkMRMLModelNode"]
        # self.trajSelector.selectNodeUponCreation = True
        # self.trajSelector.addEnabled = False
        # self.trajSelector.removeEnabled = False
        # self.trajSelector.noneEnabled = False
        # self.trajSelector.showHidden = False
        # self.trajSelector.showChildNodeTypes = False
        # self.trajSelector.setMRMLScene(slicer.mrmlScene)
        # self.trajSelector.setToolTip("Select the Trajectory to Plan:")
        # self.trajSelector.setCurrentNode(self.selectedTraj.lineModelNode)
        # self.trajSelector.setNodeTypeLabel('Trajectory',"vtkMRMLModelNode")
        self.trajSelector = qt.QComboBox()
        self.trajSelector.addItem("Trajectory 1")
        self.trajSelector.currentIndexChanged.connect(self.onTrajSelectionChange)

        actionsFormLayout.addRow("Select Trajectory to Edit: ", self.trajSelector)


        """Add Trajectory Button"""
        self.addTrajectoryButton = qt.QPushButton("Add New Trajectory")
        actionsFormLayout.addRow(self.addTrajectoryButton)

        """Delete Trajectory Button"""
        self.deleteTrajectoryButton = qt.QPushButton("Delete Current Trajectory")
        actionsFormLayout.addRow(self.deleteTrajectoryButton)

        """Save Trajectory Button"""
        self.saveTrajectoryButton = qt.QPushButton("Save Trajectory")
        saveIcon = qt.QIcon(self.dir+'/Resources/Icons/save.png')
        self.saveTrajectoryButton.setIcon(saveIcon)
        self.saveTrajectoryButton.setIconSize(qt.QSize(50,50))
        actionsFormLayout.addRow(self.saveTrajectoryButton)

        """Set Point Layout"""
        self.movePointsButtonLayout = qt.QHBoxLayout()
        self.movePointsLabelsLayout = qt.QHBoxLayout()
        """Set Target Point Button"""
        self.moveTargetToIntersectionButton = qt.QPushButton("CHANGE Target Point")
        self.moveTargetToIntersectionButton.toolTip = "Align slice intersections (hover while pressing shift may " \
                                                      "help). Click button to move target point here"
        self.moveTargetToIntersectionButton.enabled = True
        setTargetIcon = qt.QIcon(self.dir+'/Resources/Icons/setTarget.png')
        self.moveTargetToIntersectionButton.setIcon(setTargetIcon)
        self.moveTargetToIntersectionButton.setIconSize(qt.QSize(50,50))
        self.movePointsButtonLayout.addWidget(self.moveTargetToIntersectionButton)


        """Set Entry Point Button"""
        self.moveEntryToIntersectionButton = qt.QPushButton("CHANGE Entry Point")
        self.moveEntryToIntersectionButton.toolTip = "Align slice intersections (hover while pressing shift may " \
                                                     "help). Click button to move target point here"
        self.moveEntryToIntersectionButton.enabled = True
        setEntryIcon = qt.QIcon(self.dir+'/Resources/Icons/setEntry.png')
        self.moveEntryToIntersectionButton.setIcon(setEntryIcon)
        self.moveEntryToIntersectionButton.setIconSize(qt.QSize(50,50))
        x = qt.QLabel()
        x.setWordWrap(True)
        x.setText("Buttons below change the Target/Entry Point to the current slice intersection point. Hold SHIFT while hovering mouse to change slice itersection")
        self.movePointsLabelsLayout.addWidget(x)
        self.movePointsButtonLayout.addWidget(self.moveEntryToIntersectionButton)

        actionsFormLayout.addRow(self.movePointsLabelsLayout)
        actionsFormLayout.addRow(self.movePointsButtonLayout)

        self.jumpVizLabelsLayout = qt.QHBoxLayout()
        self.jumpVizButtonsLayout = qt.QHBoxLayout()

        x = qt.QLabel()
        x.setWordWrap(True)
        x.setText("Buttons below switch view to center on Target/Entry Point:")
        self.jumpVizLabelsLayout.addWidget(x)

        """Jump To Target Point Button"""
        self.jumpToTargetButton = qt.QPushButton("View Target Point")
        self.jumpToTargetButton.toolTip = "Press to see the target point in all slices"
        moveToTargetIcon = qt.QIcon(self.dir+'/Resources/Icons/moveToTarget.png')
        self.jumpToTargetButton.setIcon(moveToTargetIcon)
        self.jumpToTargetButton.setIconSize(qt.QSize(50,50))
        self.jumpVizButtonsLayout.addWidget(self.jumpToTargetButton)

        """Jump To Entry Point Button"""
        self.jumpToEntryButton = qt.QPushButton("View Entry Point")
        self.jumpToEntryButton.toolTip = "Press to see the Entry point in all slices"
        moveToEntryIcon = qt.QIcon(self.dir+'/Resources/Icons/moveToEntry.png')
        self.jumpToEntryButton.setIcon(moveToEntryIcon)
        self.jumpToEntryButton.setIconSize(qt.QSize(50,50))
        self.jumpVizButtonsLayout.addWidget(self.jumpToEntryButton)

        vizFormLayout.addRow(self.jumpVizLabelsLayout)
        vizFormLayout.addRow(self.jumpVizButtonsLayout)



        self.sliceVizLabelsLayout = qt.QHBoxLayout()
        self.sliceVizButtonsLayout = qt.QHBoxLayout()
        x = qt.QLabel()
        x.setWordWrap(True)
        x.setText("Buttons below change Slice Views:")
        self.sliceVizLabelsLayout.addWidget(x)

        """Standard View Button"""
        self.alignAxesToASCButton = qt.QPushButton("Standard")
        self.alignAxesToASCButton.toolTip = "Returns to default axial, sagittal, coronal slice views"

        standardViewIcon = qt.QIcon(self.dir + '/Resources/Icons/standard.png')
        self.alignAxesToASCButton.setIcon(standardViewIcon)
        self.alignAxesToASCButton.setIconSize(qt.QSize(50, 50))
        self.sliceVizButtonsLayout.addWidget(self.alignAxesToASCButton)

        """Look Down Trajectory Button"""
        self.alignAxesToTrajectoryButton = qt.QPushButton("Down Trajectory")
        self.alignAxesToTrajectoryButton.toolTip = "Axial view switches to down-trajectory view. " \
                                                   "Other planes rotate by same amount to remain orthogonal"

        downTrajIcon = qt.QIcon(self.dir + '/Resources/Icons/downTraj.png')
        self.alignAxesToTrajectoryButton.setIcon(downTrajIcon)
        self.alignAxesToTrajectoryButton.setIconSize(qt.QSize(50, 50))
        self.sliceVizButtonsLayout.addWidget(self.alignAxesToTrajectoryButton)

        """Oblique View Button"""
        self.obliqueViewButton = qt.QPushButton("Oblique Viewer")
        self.obliqueViewButton.toolTip = "Center point can be dragged to rotate views to oblique angle"
        # downTrajIcon = qt.QIcon(self.dir + '/Resources/Icons/downTraj.png')
        # self.alignAxesToTrajectoryButton.setIcon(downTrajIcon)
        # self.alignAxesToTrajectoryButton.setIconSize(qt.QSize(50, 50))
        self.sliceVizButtonsLayout.addWidget(self.obliqueViewButton)

        vizFormLayout.addRow(self.sliceVizLabelsLayout)
        vizFormLayout.addRow(self.sliceVizButtonsLayout)

        self.crosshairNode = slicer.util.getNode('Crosshair')  # Make sure exists
        self.crosshairPos = self.crosshairNode.SetCrosshairRAS(0, 0, 0)


        # connections
        self.addTrajectoryButton.connect('clicked(bool)', self.onAddTrajectoryButton)
        self.deleteTrajectoryButton.connect('clicked(bool)', self.onDeleteTrajectoryButton)

        self.saveTrajectoryButton.connect('clicked(bool)', self.onSaveTrajectoryButton)

        self.addTestDataButton.connect('clicked(bool)', self.onAddTestDataButton)
        self.toggleSliceIntersectionButton.connect('clicked(bool)', self.onToggleSliceIntersectionButton)

        self.moveTargetToIntersectionButton.connect('clicked(bool)', self.onMoveTargetToIntersectionButton)
        self.moveEntryToIntersectionButton.connect('clicked(bool)', self.onMoveEntryToIntersectionButton)

        self.jumpToTargetButton.connect('clicked(bool)', self.onJumpToTargetButton)
        self.jumpToEntryButton.connect('clicked(bool)', self.onJumpToEntryButton)

        self.alignAxesToTrajectoryButton.connect('clicked(bool)', self.onAlignAxesToTrajectoryButton)
        self.alignAxesToASCButton.connect('clicked(bool)', self.onAlignAxesToASCButton)
        self.obliqueViewButton.connect('clicked(bool)', self.onObliqueViewButton)


        self.addTrajFromFileButton.connect('clicked(bool)', self.onAddTrajFromFileButton)

        self.toggleRedSliceVisibilityButton.connect('clicked(bool)', self.onToggleRedSliceVisibilityButton)
        self.toggleYellowSliceVisibilityButton.connect('clicked(bool)', self.onToggleYellowSliceVisibilityButton)
        self.toggleGreenSliceVisibilityButton.connect('clicked(bool)', self.onToggleGreenSliceVisibilityButton)

        # Add vertical spacer
        self.layout.addStretch(1)

    def cleanup(self):
        pass

    def onAddTrajectoryButton(self):
        self.onSaveTrajectoryButton()  # Save the previous one, just to be safe
        self.onAlignAxesToASCButton()

        for observer in self.SelectedTrajObservers:  # Get rid of selected observer
            slicer.mrmlScene.RemoveObserver(observer)
        if self.trajSelector.count > 1:  # handle edge case where you deleted everything and add new
            self.selectedTraj.deselect()

        self.trajNumMax += 1

        self.selectedTraj = sh.SlicerTrajectoryModel(self.trajNumMax)
        self.trajList = np.append(self.trajList, self.selectedTraj)
        self.trajSelector.addItem("Trajectory " + str(self.selectedTraj.trajNum))
        self.trajSelector.setCurrentIndex(self.trajSelector.count-1)

    def onDeleteTrajectoryButton(self):
        if self.trajSelector.count:  # don't do anything if no trajectories
            del_index = self.trajSelector.currentIndex
            for observer in self.SelectedTrajObservers:  # Get rid of selected observer
                slicer.mrmlScene.RemoveObserver(observer)
            self.selectedTraj.deselect()
            self.trajSelector.removeItem(del_index)

            self.trajList[del_index].deleteNodes()
            self.trajList = np.delete(self.trajList, del_index)

    def onSaveTrajectoryButton(self):
        if not os.path.exists(self.outdir):
            os.makedirs(self.outdir)

        for i,traj in enumerate(self.trajList):
            trajoutdir = self.outdir+'/traj'+str(i+1)
            if not os.path.exists(trajoutdir):
                os.makedirs(trajoutdir)
            slicer.util.saveNode(self.selectedTraj.entryMarkupNode, trajoutdir+'/Entry.fcsv')
            slicer.util.saveNode(self.selectedTraj.targetMarkupNode, trajoutdir+'/Target.fcsv')
            slicer.util.saveNode(self.selectedTraj.lineModelNode, trajoutdir + '/line.vtk')
            slicer.util.saveNode(self.toolMeshModel.transform_node, trajoutdir+'/needle.h5')

        print('Trajectories saved to '+ self.outdir)

    def onAddTrajFromFileButton(self): ## TODO FINISH ADD TRAJ FROM FILE
        file_success = False
        ## Read file
        if file_success:
            tpos = np.array([0.0, 0.0, 0.0])
            epos = np.array([0.0, 0.0, 0.0])
            # Info from file
            self.onAddTrajectoryButton()
            self.selectedTraj.targetMarkupNode.SetNthFiducialPosition(0, tpos[0], tpos[1], tpos[2])
            self.selectedTraj.entryMarkupNode.SetNthFiducialPosition(0, epos[0], epos[1], epos[2])

    def onAddTestDataButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.addTestData(self.test_volume_data_filename, self.test_ct_directory)
        slicer.util.selectModule('InjectionTrajectoryPlanner')  # Switch back after going to DICOM module

    def onToggleSliceIntersectionButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.toggleSliceIntersection()

    def onToggleRedSliceVisibilityButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.toggleSliceVisibility('Red')

    def onToggleYellowSliceVisibilityButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.toggleSliceVisibility('Yellow')

    def onToggleGreenSliceVisibilityButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.toggleSliceVisibility('Green')

    def onMoveTargetToIntersectionButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.moveTargetToIntersectionButton(self.selectedTraj.targetMarkupNode)

    def onMoveEntryToIntersectionButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.moveEntryToIntersectionButton(self.selectedTraj.entryMarkupNode)

    def onJumpToTargetButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.jumpToMarkup(self.selectedTraj.targetMarkupNode)

    def onJumpToEntryButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.jumpToMarkup(self.selectedTraj.entryMarkupNode)

    def onAlignAxesToTrajectoryButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.alignAxesWithTrajectory(self.selectedTraj.targetMarkupNode, self.selectedTraj.entryMarkupNode)
        self.onJumpToTargetButton()  # return crosshair to target point

        layoutManager = slicer.app.layoutManager()
        threeDWidget = layoutManager.threeDWidget(0)
        threeDView = threeDWidget.threeDView()
        threeDView.resetFocalPoint()
        self.downAxisBool = True

    def onAlignAxesToASCButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        logic.resetAxesToASC(self.selectedTraj.targetMarkupNode)
        self.onJumpToTargetButton()  # return crosshair to target point

        layoutManager = slicer.app.layoutManager()
        threeDWidget = layoutManager.threeDWidget(0)
        threeDView = threeDWidget.threeDView()
        threeDView.resetFocalPoint()
        if hasattr(self, 'downAxisBool') and self.downAxisBool:
            self.DATrajectoryMarkupNode.SetNthFiducialVisibility(0, False)
        self.downAxisBool = False

    def onObliqueViewButton(self):
        logic = InjectionTrajectoryPlannerLogic()
        self.obliqueRotator = sh.SlicerTrajectoryModel(-1, selected_bool=True)
        self.obliqueRotator.lineModelNode.SetName('Drag')
        self.obliqueRotator.targetMarkupNode.SetName('Drag')
        self.obliqueRotator.targetMarkupNode.SetNthFiducialLabel(0, 'Drag')
        self.obliqueRotator.entryMarkupNode.HideFromEditorsOn()
        self.obliqueRotator.entryMarkupNode.GetNthDisplayNode(0).VisibilityOff()
        redSliceViewPoint = sh.arrayFromVTKMatrix(self.redSliceNode.GetSliceToRAS())[0:3, 3]
        redSliceNormal = sh.arrayFromVTKMatrix(self.redSliceNode.GetSliceToRAS())[0:3, 2]

        self.obliqueRotator.targetMarkupNode.SetNthFiducialPosition(0,
                                                           redSliceViewPoint[0],
                                                           redSliceViewPoint[1],
                                                           redSliceViewPoint[2], )
        self.obliqueRotator.entryMarkupNode.SetNthFiducialPosition(0,
                                                           redSliceViewPoint[0],
                                                           redSliceViewPoint[1],
                                                           redSliceViewPoint[2], )
        self.obliqueRotatorLastStaticPos = redSliceViewPoint
        self.obliqueRotatorLastNormal = redSliceNormal
        self.obliqueRotator.targetMarkupNode.AddObserver(slicer.vtkMRMLMarkupsNode.PointModifiedEvent,
                                          self.obliqueMarkupModifiedCallback)
        self.obliqueRotator.targetMarkupNode.AddObserver(slicer.vtkMRMLMarkupsNode.PointEndInteractionEvent,
                                         self.obliqueMarkupEndInteractionCallback)

    def onTrajSelectionChange(self, index):
        self.onAlignAxesToASCButton()
        for observer in self.SelectedTrajObservers:  # Get rid of selected observer
            slicer.mrmlScene.RemoveObserver(observer)
        self.selectedTraj.deselect()
        if index >=0:
            self.selectedTraj = self.trajList[index]
            self.addSelectedTrajObservers(self.selectedTraj)
            self.selectedTraj.select()
            self.onJumpToTargetButton()

    def targetMarkupModifiedCallback(self, caller, event):
        pass
    #     pos1 = [0.0, 0.0, 0.0]
    #     self.targetMarkupNode.GetNthFiducialPosition(0, pos1)
    #     self.selectedTraj.line.SetPoint1(pos1)
    #     self.selectedTraj.line.Update()
    #     self.UpdateToolModel()
    #
    def entryMarkupModifiedCallback(self, caller, event):
        pass
    #     pos2 = [0.0, 0.0, 0.0]
    #     self.entryMarkupNode.GetNthFiducialPosition(0, pos2)
    #     self.selectedTraj.line.SetPoint2(pos2)
    #     self.selectedTraj.line.Update()
    #     self.UpdateToolModel()

    def obliqueMarkupModifiedCallback(self, caller, event):
        # self.logic.obliqePivotView(self.obliqueRotator.targetMarkupNode, self.obliqueRotatorLastStaticPos, self.obliqueRotatorLastNormal)
        pass
    def obliqueMarkupEndInteractionCallback(self, caller, event):
        self.logic.obliqePivotView(self.obliqueRotator.targetMarkupNode, self.obliqueRotatorLastStaticPos, self.obliqueRotatorLastNormal)

        # print('obmee')
        redSliceViewPoint = sh.arrayFromVTKMatrix(self.redSliceNode.GetSliceToRAS())[0:3, 3]
        redSliceNormal = sh.arrayFromVTKMatrix(self.redSliceNode.GetSliceToRAS())[0:3, 2]

        self.obliqueRotator.targetMarkupNode.SetNthFiducialPosition(0,
                                                                    redSliceViewPoint[0],
                                                                    redSliceViewPoint[1],
                                                                    redSliceViewPoint[2], )
        self.obliqueRotator.entryMarkupNode.SetNthFiducialPosition(0,
                                                                   redSliceViewPoint[0],
                                                                   redSliceViewPoint[1],
                                                                   redSliceViewPoint[2], )
        self.obliqueRotatorLastStaticPos = redSliceViewPoint
        self.obliqueRotatorLastNormal = redSliceNormal

###TODO OBLIQUE VIEWS ARE DAT BUT UPDATES IN RT AND REVERTS WHEN YOU LET GO
###TODO ADD CLEAN OUTPUT WHERE ALL IN ONE MARKUPS LIST

    def targetMarkupEndInteractionCallback(self, caller, event):
        if hasattr(self, 'downAxisBool') and self.downAxisBool:
            DAT_pos = np.array([0.0, 0.0, 0.0])
            entry_pos = np.array([0.0, 0.0, 0.0])
            target_pos = np.array([0.0, 0.0, 0.0])
            self.DATrajectoryMarkupNode.GetNthFiducialPosition(0, DAT_pos)
            self.selectedTraj.entryMarkupNode.GetNthFiducialPosition(0, entry_pos)
            self.selectedTraj.targetMarkupNode.GetNthFiducialPosition(0, target_pos)

            old_entry_to_DAT = (DAT_pos - entry_pos)
            norm_old_entry_to_DAT = np.linalg.norm(old_entry_to_DAT)
            new_traj = target_pos - entry_pos
            norm_new_traj = np.linalg.norm(new_traj)
            new_entry_to_DAT = new_traj / norm_new_traj * norm_old_entry_to_DAT
            new_DAT_pos = entry_pos + new_entry_to_DAT
            self.DATrajectoryMarkupNode.SetNthFiducialPosition(
                0, new_DAT_pos[0], new_DAT_pos[1], new_DAT_pos[2])
            self.logic.alignAxesWithTrajectory(self.DATrajectoryMarkupNode, self.entryMarkupNode)

    def entryMarkupEndInteractionCallback(self, caller, event):
        if hasattr(self, 'downAxisBool') and self.downAxisBool:
            DAT_pos = np.array([0.0, 0.0, 0.0])
            entry_pos = np.array([0.0, 0.0, 0.0])
            target_pos = np.array([0.0, 0.0, 0.0])
            self.DATrajectoryMarkupNode.GetNthFiducialPosition(0, DAT_pos)
            self.selectedTraj.entryMarkupNode.GetNthFiducialPosition(0, entry_pos)
            self.selectedTraj.targetMarkupNode.GetNthFiducialPosition(0, target_pos)

            old_target_to_DAT = (DAT_pos - target_pos)
            norm_old_target_to_DAT = np.linalg.norm(old_target_to_DAT)
            new_traj = entry_pos - target_pos
            norm_new_traj = np.linalg.norm(new_traj)
            new_target_to_DAT = new_traj / norm_new_traj * norm_old_target_to_DAT
            new_DAT_pos = target_pos + new_target_to_DAT
            self.DATrajectoryMarkupNode.SetNthFiducialPosition(
                0, new_DAT_pos[0], new_DAT_pos[1], new_DAT_pos[2])
            self.logic.alignAxesWithTrajectory(self.DATrajectoryMarkupNode, self.entryMarkupNode)

    def redSliceModifiedCallback(self, caller, event):
        if hasattr(self, 'downAxisBool') and self.downAxisBool and \
                self.redSliceNode.GetSliceOffset() is not self.lastRedSliceOffset:
            # Last one checks to see if slice level actually changed
            p_target = np.array([0.0, 0.0, 0.0])
            p_Entry = np.array([0.0, 0.0, 0.0])
            self.selectedTraj.targetMarkupNode.GetNthFiducialPosition(0, p_target)
            self.selectedTraj.entryMarkupNode.GetNthFiducialPosition(0, p_Entry)
            traj = (p_target-p_Entry)
            unit_traj = traj/np.linalg.norm(traj)
            redSliceViewPoint = sh.arrayFromVTKMatrix(self.redSliceNode.GetSliceToRAS())[0:3, 3]

            in_traj = 0 < np.dot((redSliceViewPoint-p_Entry)/np.linalg.norm(traj), unit_traj) < 1
            if in_traj:
                self.lastDATFromProjection = True
                self.DATrajectoryMarkupNode.SetNthFiducialPosition(0,
                                                                   redSliceViewPoint[0],
                                                                   redSliceViewPoint[1],
                                                                   redSliceViewPoint[2],)
                self.DATrajectoryMarkupNode.SetNthFiducialVisibility(0, True)
            else:
                self.DATrajectoryMarkupNode.SetNthFiducialVisibility(0, False)
        else:
            self.DATrajectoryMarkupNode.SetNthFiducialVisibility(0, False)
        self.lastRedSliceOffset = self.redSliceNode.GetSliceOffset()

    # TODO: Make not just read slice node but many screens
    def DATPointEndInteractionCallback(self, caller, event):
        if hasattr(self, 'downAxisBool') and self.downAxisBool:
            DA_pos = np.array([0.0, 0.0, 0.0])
            entry_pos = np.array([0.0, 0.0, 0.0])
            target_pos = np.array([0.0, 0.0, 0.0])
            self.DATrajectoryMarkupNode.GetNthFiducialPosition(0, DA_pos)
            self.selectedTraj.entryMarkupNode.GetNthFiducialPosition(0, entry_pos)
            self.selectedTraj.targetMarkupNode.GetNthFiducialPosition(0, target_pos)

            old_traj = (target_pos - entry_pos)
            norm_old_traj = np.linalg.norm(old_traj)
            new_reference = DA_pos - target_pos
            norm_new_reference = np.linalg.norm(new_reference)
            new_traj = new_reference / norm_new_reference * norm_old_traj
            new_entry_pos = target_pos + new_traj
            self.selectedTraj.entryMarkupNode.SetNthFiducialPosition(0,
                                                        new_entry_pos[0],
                                                        new_entry_pos[1],
                                                        new_entry_pos[2])
            self.logic.alignAxesWithTrajectory(self.DATrajectoryMarkupNode, self.selectedTraj.entryMarkupNode)
    # # noinspection PyUnusedLocal
    # def DATrajectoryMarkupModifiedCallback(self, caller, event):
    #     if self.downAxisBool:


    def UpdateToolModel(self):
        transform = np.eye(4)
        target_pos = np.array([0.0, 0.0, 0.0])
        entry_pos = np.array([0.0, 0.0, 0.0])
        self.selectedTraj.targetMarkupNode.GetNthFiducialPosition(0, target_pos)
        self.selectedTraj.entryMarkupNode.GetNthFiducialPosition(0, entry_pos)
        traj_vec = target_pos - entry_pos
        z_vec = traj_vec
        x_vec = np.array([-z_vec[1], z_vec[0], 0])
        y_vec = np.cross(z_vec, x_vec)
        x_vec = x_vec / max(np.linalg.norm(x_vec), 1e-8)  # normalize, with protection against zero or nearzero vector
        y_vec = y_vec / max(np.linalg.norm(y_vec), 1e-8)
        z_vec = z_vec / max(np.linalg.norm(z_vec), 1e-8)

        transform[0:3, 0] = x_vec
        transform[0:3, 1] = y_vec
        transform[0:3, 2] = z_vec
        transform[0:3, 3] = target_pos
        sh.updateTransformMatrixFromArray(self.toolMeshModel.transform_node, transform)

        entry_transform = np.eye(4)
        entry_transform[0:3, 0] = x_vec
        entry_transform[0:3, 1] = y_vec
        entry_transform[0:3, 2] = z_vec
        entry_transform[0:3, 3] = entry_pos

        sh.updateTransformMatrixFromArray(self.entry_transform_node, entry_transform)

        hand_eye_transform = np.eye(4)
        hand_eye_transform = sh.arrayFromTransformMatrix(self.hand_eye_node)
        robot_ee_target_transform = np.matmul(np.linalg.inv(hand_eye_transform), transform)
        sh.updateTransformMatrixFromArray(self.robot_ee_target, robot_ee_target_transform)

        robot_ee_entry_transform = np.matmul(np.linalg.inv(hand_eye_transform), entry_transform)
        sh.updateTransformMatrixFromArray(self.robot_ee_entry, robot_ee_entry_transform)

    def addSelectedTrajObservers(self, selectedTraj):
        obs_list = []
        obs_list.append(selectedTraj.targetMarkupNode.AddObserver(slicer.vtkMRMLMarkupsNode.PointModifiedEvent,
                                                       self.targetMarkupModifiedCallback))
        obs_list.append(selectedTraj.entryMarkupNode.AddObserver(slicer.vtkMRMLMarkupsNode.PointModifiedEvent,
                                                      self.entryMarkupModifiedCallback))
        obs_list.append(selectedTraj.targetMarkupNode.AddObserver(slicer.vtkMRMLMarkupsNode.PointEndInteractionEvent,
                                          self.targetMarkupEndInteractionCallback))
        obs_list.append(selectedTraj.entryMarkupNode.AddObserver(slicer.vtkMRMLMarkupsNode.PointEndInteractionEvent,
                                         self.entryMarkupEndInteractionCallback))
        self.SelectedTrajObservers = obs_list


    # def projectCurrentTrajPoint(self):
    #     if hasattr(self, 'downAxisBool') and self.downAxisBool:
    #         p_target = np.array([0.0, 0.0, 0.0])
    #         p_Entry = np.array([0.0, 0.0, 0.0])
    #         self.targetMarkupNode.GetNthFiducialPosition(0, p_target)
    #         self.EntryMarkupNode.GetNthFiducialPosition(0, p_Entry)
    #         traj = (p_target-p_Entry)
    #         unit_traj = traj/np.linalg.norm(traj)
    #         redSliceNode = slicer.util.getNode('vtkMRMLSliceNodeRed')
    #         redSliceViewPoint = sh.arrayFromVTKMatrix(redSliceNode.GetSliceToRAS())[0:3, 3]
    #
    #         in_traj = 0 < np.dot((redSliceViewPoint-p_Entry)/np.linalg.norm(traj), unit_traj) < 1
    #         if in_traj:
    #             self.lastDATFromProjection = True
    #             self.DATrajectoryMarkupNode.SetNthFiducialPosition(0,
    #                                                                redSliceViewPoint[0],
    #                                                                redSliceViewPoint[1],
    #                                                                redSliceViewPoint[2],)
    #             self.DATrajectoryMarkupNode.SetNthFiducialVisibility(0, True)
    #         else:
    #             self.DATrajectoryMarkupNode.SetNthFiducialVisibility(0, False)
    #     else:
    #         self.DATrajectoryMarkupNode.SetNthFiducialVisibility(0, False)

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
        # noinspection PyUnresolvedReferences
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

    def toggleSliceVisibility(self, name):
        layoutManager = slicer.app.layoutManager()
        for sliceViewName in layoutManager.sliceViewNames():
            if sliceViewName == name:
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
        pos = [0.0, 0.0, 0.0]
        MarkupNode.GetNthFiducialPosition(0, pos)

        for name in ['Red', 'Yellow', 'Green']:
            sliceNode = slicer.app.layoutManager().sliceWidget(name).mrmlSliceNode()
            sliceNode.JumpSlice(pos[0], pos[1], pos[2])

    def obliqePivotView(self, MarkupNode, p_last_static, normal_last_static):
        pos = [0.0, 0.0, 0.0]
        MarkupNode.GetNthFiducialPosition(0, pos)

        redSliceNode = slicer.util.getNode('vtkMRMLSliceNodeRed')
        yellowSliceNode = slicer.util.getNode('vtkMRMLSliceNodeYellow')
        greenSliceNode = slicer.util.getNode('vtkMRMLSliceNodeGreen')

        redSliceToRAS = redSliceNode.GetSliceToRAS()
        yellowSliceToRAS = yellowSliceNode.GetSliceToRAS()
        greenSliceToRAS = greenSliceNode.GetSliceToRAS()

        v = np.array(pos) - p_last_static
        print('v: ', v)
        n = normal_last_static
        print('n: ', n)

        # v_norm = np.linalg.norm(v)
        # proj = (np.dot(n, v) / v_norm ** 2) * v

        proj = n+v

        sliceNormal = proj / max(np.linalg.norm(proj), 1e-8)
        slicePosition = p_last_static

        # v_norm = np.linalg.norm(v)
        # new_pos = p_last_static + (np.dot(n, v) / v_norm ** 2) * v
        # MarkupNode.SetNthFiducialPosition(0, new_pos[0], new_pos[1], new_pos[2])


        setSlicePoseFromSliceNormalAndPosition(redSliceNode, sliceNormal, slicePosition)

        swap_yellow = vtk.vtkMatrix4x4()
        swap_yellow.SetElement(1, 1, 0)
        swap_yellow.SetElement(2, 1, 1)
        swap_yellow.SetElement(1, 2, 1)
        swap_yellow.SetElement(2, 2, 0)
        vtk.vtkMatrix4x4().Multiply4x4(redSliceToRAS, swap_yellow, yellowSliceToRAS)
        yellowSliceNode.UpdateMatrices()

        swap_green = vtk.vtkMatrix4x4()
        swap_green.SetElement(0, 0, 0)
        swap_green.SetElement(1, 0, -1)
        swap_green.SetElement(1, 1, 0)
        swap_green.SetElement(2, 1, 1)
        swap_green.SetElement(0, 2, -1)
        swap_green.SetElement(2, 2, 0)
        vtk.vtkMatrix4x4().Multiply4x4(redSliceToRAS, swap_green, greenSliceToRAS)

        greenSliceNode.UpdateMatrices()


    def alignAxesWithTrajectory(self, targetMarkupNode, EntryMarkupNode):
        import numpy as np

        redSliceNode = slicer.util.getNode('vtkMRMLSliceNodeRed')
        yellowSliceNode = slicer.util.getNode('vtkMRMLSliceNodeYellow')
        greenSliceNode = slicer.util.getNode('vtkMRMLSliceNodeGreen')

        redSliceToRAS = redSliceNode.GetSliceToRAS()
        yellowSliceToRAS = yellowSliceNode.GetSliceToRAS()
        greenSliceToRAS = greenSliceNode.GetSliceToRAS()

        p_target = np.array([0.0, 0.0, 0.0])
        p_Entry = np.array([0.0, 0.0, 0.0])

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
        # yellowSliceToRAS.SetElement(0, 3, p_target[0])
        # yellowSliceToRAS.SetElement(1, 3, p_target[1])
        # yellowSliceToRAS.SetElement(2, 3, p_target[2])
        yellowSliceNode.UpdateMatrices()

        swap_green = vtk.vtkMatrix4x4()
        swap_green.SetElement(0, 0, 0)
        swap_green.SetElement(1, 0, -1)
        swap_green.SetElement(1, 1, 0)
        swap_green.SetElement(2, 1, 1)
        swap_green.SetElement(0, 2, -1)
        swap_green.SetElement(2, 2, 0)
        vtk.vtkMatrix4x4().Multiply4x4(redSliceToRAS, swap_green, greenSliceToRAS)
        # greenSliceToRAS.SetElement(0, 3, p_target[0])
        # greenSliceToRAS.SetElement(1, 3, p_target[1])
        # greenSliceToRAS.SetElement(2, 3, p_target[2])

        greenSliceNode.UpdateMatrices()

        # # Turn on trajectory manipulation point
        # for i in range(DATrajectoryMarkupNode.GetNumberOfMarkups()):
        #     DATrajectoryMarkupNode.SetNthFiducialVisibility(i, True)
        # # Turn off trajectory manipulation point
        # for i in range(DATrajectoryMarkupNode.GetNumberOfMarkups()):
        #     DATrajectoryMarkupNode.SetNthFiducialVisibility(i, False)

    def resetAxesToASC(self, targetMarkupNode):
        import numpy as np
        redSliceNode = slicer.util.getNode('vtkMRMLSliceNodeRed')
        yellowSliceNode = slicer.util.getNode('vtkMRMLSliceNodeYellow')
        greenSliceNode = slicer.util.getNode('vtkMRMLSliceNodeGreen')
        redSliceNode.SetOrientationToAxial()
        yellowSliceNode.SetOrientationToSagittal()
        greenSliceNode.SetOrientationToCoronal()
        p_target = np.array([0.0, 0.0, 0.0])
        targetMarkupNode.GetNthFiducialPosition(0, p_target)



        # redSliceToRAS = redSliceNode.GetSliceToRAS()
        # redSliceToRAS.SetElement(0, 3, p_target[0])
        # redSliceToRAS.SetElement(1, 3, p_target[1])
        # redSliceToRAS.SetElement(2, 3, p_target[2])
        # yellowSliceToRAS = yellowSliceNode.GetSliceToRAS()
        # yellowSliceToRAS.SetElement(0, 3, p_target[0])
        # yellowSliceToRAS.SetElement(1, 3, p_target[1])
        # yellowSliceToRAS.SetElement(2, 3, p_target[2])
        # greenSliceToRAS = greenSliceNode.GetSliceToRAS()
        # greenSliceToRAS.SetElement(0, 3, p_target[0])
        # greenSliceToRAS.SetElement(1, 3, p_target[1])
        # greenSliceToRAS.SetElement(2, 3, p_target[2])


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
        # No tests for now
        self.delayDisplay('Test passed!')
