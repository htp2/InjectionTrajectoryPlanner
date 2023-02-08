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

# this is just to prevent pylance from complaining about np.cross not always returning
# https://github.com/microsoft/pylance-release/issues/3277


def np_cross(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.cross(a, b)


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
        sliceNormalStandardized = [-sliceNormal[0], -
                                   sliceNormal[1], -sliceNormal[2]]
    # Compute slice axes
    sliceNormalViewUpAngle = vtk.vtkMath.AngleBetweenVectors(
        sliceNormalStandardized, defaultViewUpDirection)
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

# CM_Path
#


class CM_Path(ScriptedLoadableModule):
    """Uses ScriptedLoadableModule base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = "CM_Path"
        self.parent.categories = ["Planning"]
        self.parent.dependencies = []
        self.parent.contributors = ["Henry Phalen (Johns Hopkins University)"]
        self.parent.helpText = """
This module can be used to plan and visualize a path for a continuum manipulator.
"""
        self.parent.helpText += self.getDefaultModuleDocumentationLink()
        self.parent.acknowledgementText = """
Developed by Henry Phalen, a PhD student at Johns Hopkins University.
"""


# noinspection PyAttributeOutsideInit,PyMethodMayBeStatic
class CM_PathWidget(ScriptedLoadableModuleWidget):
    """Uses ScriptedLoadableModuleWidget base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """

    def setup(self):
        ScriptedLoadableModuleWidget.setup(self)
        self.dir = os.path.dirname(__file__)

        # Initialize Useful Parameters
        self.logic = CM_PathLogic()

        # Setup main layout tabs (collapsible buttons)
        actionsCollapsibleButton = ctk.ctkCollapsibleButton()
        actionsCollapsibleButton.text = "Actions"
        self.layout.addWidget(actionsCollapsibleButton)
        actionsFormLayout = qt.QFormLayout(actionsCollapsibleButton)

        vizCollapsibleButton = ctk.ctkCollapsibleButton()
        vizCollapsibleButton.text = "Visualization"
        self.layout.addWidget(vizCollapsibleButton)
        vizFormLayout = qt.QFormLayout(vizCollapsibleButton)

        parametersCollapsibleButton = ctk.ctkCollapsibleButton()
        parametersCollapsibleButton.text = "Parameters"
        self.layout.addWidget(parametersCollapsibleButton)
        parametersCollapsibleButton.setChecked(True)  # closes by default
        parametersFormLayout = qt.QFormLayout(parametersCollapsibleButton)

        # node combo box for plane markup node
        self.planeSelector = slicer.qMRMLNodeComboBox()
        self.planeSelector.nodeTypes = ["vtkMRMLMarkupsPlaneNode"]
        self.planeSelector.selectNodeUponCreation = True
        self.planeSelector.addEnabled = True
        self.planeSelector.removeEnabled = True
        self.planeSelector.noneEnabled = True
        self.planeSelector.showHidden = False
        self.planeSelector.showChildNodeTypes = False
        self.planeSelector.setMRMLScene(slicer.mrmlScene)
        self.planeSelector.setToolTip("Pick the plane markup node.")
        parametersFormLayout.addRow("Plane Markup Node: ", self.planeSelector)

        # node combo box for closed curve markup node
        self.curveSelector = slicer.qMRMLNodeComboBox()
        self.curveSelector.nodeTypes = ["vtkMRMLMarkupsClosedCurveNode"]
        self.curveSelector.selectNodeUponCreation = True
        self.curveSelector.addEnabled = True
        self.curveSelector.removeEnabled = True
        self.curveSelector.noneEnabled = True
        self.curveSelector.showHidden = False
        self.curveSelector.showChildNodeTypes = False
        self.curveSelector.setMRMLScene(slicer.mrmlScene)
        self.curveSelector.setToolTip("Pick the closed curve markup node.")
        parametersFormLayout.addRow(
            "Closed Curve Markup Node: ", self.curveSelector)

        # node combo box for fiducial markup node
        self.fiducialSelector = slicer.qMRMLNodeComboBox()
        self.fiducialSelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
        self.fiducialSelector.selectNodeUponCreation = True
        self.fiducialSelector.addEnabled = True
        self.fiducialSelector.removeEnabled = True
        self.fiducialSelector.noneEnabled = True
        self.fiducialSelector.showHidden = False
        self.fiducialSelector.showChildNodeTypes = False
        self.fiducialSelector.setMRMLScene(slicer.mrmlScene)
        self.fiducialSelector.setToolTip("Pick the fiducial markup node.")
        parametersFormLayout.addRow(
            "Fiducial Markup Node: ", self.fiducialSelector)

        # node combo box for line markup node
        self.lineSelector = slicer.qMRMLNodeComboBox()
        self.lineSelector.nodeTypes = ["vtkMRMLMarkupsLineNode"]
        self.lineSelector.selectNodeUponCreation = True
        self.lineSelector.addEnabled = True
        self.lineSelector.removeEnabled = True
        self.lineSelector.noneEnabled = True
        self.lineSelector.showHidden = False
        self.lineSelector.showChildNodeTypes = False
        self.lineSelector.setMRMLScene(slicer.mrmlScene)
        self.lineSelector.setToolTip("Pick the line markup node.")
        parametersFormLayout.addRow("Line Markup Node: ", self.lineSelector)

        # node combo box for volume node
        self.volumeSelector = slicer.qMRMLNodeComboBox()
        self.volumeSelector.nodeTypes = ["vtkMRMLScalarVolumeNode"]
        self.volumeSelector.selectNodeUponCreation = True
        self.volumeSelector.addEnabled = True
        self.volumeSelector.removeEnabled = True
        self.volumeSelector.noneEnabled = True
        self.volumeSelector.showHidden = False
        self.volumeSelector.showChildNodeTypes = False
        self.volumeSelector.setMRMLScene(slicer.mrmlScene)
        self.volumeSelector.setToolTip("Pick the volume.")
        parametersFormLayout.addRow("Volume: ", self.volumeSelector)

        # node combo box for plan trajectory node (open curve)
        self.planTrajectorySelector = slicer.qMRMLNodeComboBox()
        self.planTrajectorySelector.nodeTypes = ["vtkMRMLMarkupsCurveNode"]
        self.planTrajectorySelector.selectNodeUponCreation = True
        self.planTrajectorySelector.addEnabled = True
        self.planTrajectorySelector.removeEnabled = True
        self.planTrajectorySelector.noneEnabled = True
        self.planTrajectorySelector.showHidden = False
        self.planTrajectorySelector.showChildNodeTypes = False
        self.planTrajectorySelector.setMRMLScene(slicer.mrmlScene)
        self.planTrajectorySelector.setToolTip(
            "Pick the plan trajectory node.")
        parametersFormLayout.addRow(
            "Plan Trajectory: ", self.planTrajectorySelector)

        # make an entry for a number which is defaulted to 3.0 called insertion depth per pass
        self.insertionDepthPerPass = qt.QDoubleSpinBox()
        self.insertionDepthPerPass.value = 3.0
        self.insertionDepthPerPass.singleStep = 0.1
        self.insertionDepthPerPass.minimum = 0.1
        self.insertionDepthPerPass.maximum = 30.0
        self.insertionDepthPerPass.setToolTip(
            "Insertion depth per pass in mm.")
        parametersFormLayout.addRow(
            "Insertion Depth Per Pass: ", self.insertionDepthPerPass)

        # make an entry for a number which is defaulted to 10 called points per pass
        self.pointsPerPass = qt.QSpinBox()
        self.pointsPerPass.value = 10
        self.pointsPerPass.singleStep = 1
        self.pointsPerPass.minimum = 1
        self.pointsPerPass.maximum = 100
        self.pointsPerPass.setToolTip("Number of points per pass.")
        parametersFormLayout.addRow("Points Per Pass: ", self.pointsPerPass)
        
        self.axial = True
        # add a checkbox to the actions tab to toggle between axial and cutplane view
        self.switchButton = qt.QPushButton("Switch Axial/CutPlane View")
        self.switchButton.toolTip = "Switch between axial and cutplane view."
        self.switchButton.enabled = True
        actionsFormLayout.addRow(self.switchButton)

        # add a run button to the actions tab
        self.runButton = qt.QPushButton("Run")
        self.runButton.toolTip = "Run the algorithm."
        self.runButton.enabled = True
        actionsFormLayout.addRow(self.runButton)

        # add button to toggle live update
        self.live_update = False
        self.liveUpdateButton = qt.QPushButton("Toggle Live Update")
        self.liveUpdateButton.toolTip = "Toggle live update."
        self.liveUpdateButton.enabled = True
        actionsFormLayout.addRow(self.liveUpdateButton)

        # connections
        self.runButton.connect('clicked(bool)', self.onRun)
        self.switchButton.connect('clicked(bool)', self.onSwitch)
        self.liveUpdateButton.connect('clicked(bool)', self.onLiveUpdate)

        # Add vertical spacer
        self.layout.addStretch(1)       

    def onRun(self):
        logic = CM_PathLogic()
        logic.run(self.planeSelector.currentNode(), self.curveSelector.currentNode(), self.fiducialSelector.currentNode(), 
            self.lineSelector.currentNode(), self.planTrajectorySelector.currentNode(), self.insertionDepthPerPass.value, self.pointsPerPass.value)
    def onSwitch(self):
        logic = CM_PathLogic()
        if self.axial:
            self.axial = False
            logic.switch_cutplane(
                self.planeSelector.currentNode(), self.curveSelector.currentNode())
        else:
            self.axial = True
            logic.switch_axial()
    
    def onLiveUpdate(self):
        self.live_update = not self.live_update
        print("Live Update: ", self.live_update)
        if self.live_update:
            self.curveObserver = self.curveSelector.currentNode().AddObserver(
                slicer.vtkMRMLMarkupsFiducialNode.PointModifiedEvent, self.onCurveModified)
            self.fiducialObserver = self.fiducialSelector.currentNode().AddObserver(
                slicer.vtkMRMLMarkupsFiducialNode.PointModifiedEvent, self.onFiducialModified)
            self.insertionDepthPerPass.connect('valueChanged(double)', self.onRun)
            self.pointsPerPass.connect('valueChanged(int)', self.onRun)
            # change the button text
            self.liveUpdateButton.setText("Disable Live Update")
        else:
            self.curveSelector.currentNode().RemoveObserver(self.curveObserver)
            self.fiducialSelector.currentNode().RemoveObserver(self.fiducialObserver)
            self.insertionDepthPerPass.disconnect('valueChanged(double)', self.onRun)
            self.pointsPerPass.disconnect('valueChanged(int)', self.onRun)
            # change the button text
            self.liveUpdateButton.setText("Enable Live Update")

    def onCurveModified(self, caller, event):
        print("Curve Modified")
        if self.live_update:
            self.onRun()
    
    def onFiducialModified(self, caller, event):
        print("Fiducial Modified")
        if self.live_update:
            self.onRun()

    def cleanup(self):
        pass


#
# CM_PathLogic
#

# noinspection PyMethodMayBeStatic
class CM_PathLogic(ScriptedLoadableModuleLogic):
    """This class should implement all the actual
  computation done by your module.  The interface
  should be such that other python code can import
  this class and make use of the functionality without
  requiring an instance of the Widget.
  Uses ScriptedLoadableModuleLogic base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """

    def run(self, plane, curve, fiducial, line, planTrajectory, insertion_depth_per_pass, points_per_pass):
        # find closest point on curve to first point in fiducial
        print("start run")

        fiducial_point = np.array((0.0, 0.0, 0.0))
        fiducial.GetNthFiducialPosition(0, fiducial_point)

        # curve is a vtkMRMLMarkupsClosedCurveNode

        # make a copy of curve called with same name and _fine appended
        # make new vtkMRMLMarkupsClosedCurveNode
        fine = slicer.vtkMRMLMarkupsClosedCurveNode()
        fine.SetName(curve.GetName() + "_fine")
        fine.Copy(curve)

        fine.ResampleCurveWorld(1.0)  # 1mm spacing
        # get the points from the fine curve
        spline_points = fine.GetCurveWorld()
        spline_points_array = spline_points.GetPoints()

        # find the closest point on the spline to the fiducial point
        closest_point = np.array((0.0, 0.0, 0.0))
        closest_point_index = 0

        for i in range(spline_points_array.GetNumberOfPoints()):
            point = np.array((0.0, 0.0, 0.0))
            spline_points_array.GetPoint(i, point)
            if i == 0:
                closest_point = point
            else:
                if np.linalg.norm(point - fiducial_point) < np.linalg.norm(closest_point - fiducial_point):
                    closest_point = point
                    closest_point_index = i

        # make fiducial point a vtkvector3d
        fiducial_point = vtk.vtkVector3d(
            fiducial_point[0], fiducial_point[1], fiducial_point[2])
        # make closest point a vtkvector3d
        closest_point = vtk.vtkVector3d(
            closest_point[0], closest_point[1], closest_point[2])
        line.RemoveAllControlPoints()

        line.AddControlPointWorld(fiducial_point)
        line.AddControlPointWorld(closest_point)
        print("Closest point is " + str(closest_point) +
              " at index " + str(closest_point_index))

        plane_normal = np.array((0.0, 0.0, 0.0))
        plane.GetNormal(plane_normal)

        fiducial_point_np = np.array(
            (fiducial_point[0], fiducial_point[1], fiducial_point[2]))
        insertion_position_np = np.array(
            (closest_point[0], closest_point[1], closest_point[2]))
        insertion_normal_np = insertion_position_np - fiducial_point_np
        insertion_normal_np = insertion_normal_np / np.linalg.norm(insertion_normal_np)
        self.plan_traj(insertion_position_np, insertion_normal_np,
                       plane_normal, spline_points_array, planTrajectory, insertion_depth_per_pass, points_per_pass)
        print("end run")

    def find_points_on_arc(self, insertion_length, insertion_position, th_min, th_max, R, num_points=10):
        # make arc with center at insertion_position and radius insertion_length and start point at pos_after_insertion with angle of max_rot_angle_at_insertion
        r = insertion_length
        n = num_points
        theta = np.linspace(th_min, th_max, n)
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        z = np.zeros(n)
        points = np.array([x, y, z]).T

        # rotate points by R and add to insertion_position
        points = (R @ points.T).T + insertion_position
        return points, theta

    def point_inside_polygon(self, x, y, poly, include_edges=True):
        # '''
        # Test if point (x,y) is inside polygon poly.

        # poly is N-vertices polygon defined as
        # [(x1,y1),...,(xN,yN)] or [(x1,y1),...,(xN,yN),(x1,y1)]
        # (function works fine in both cases)

        # Geometrical idea: point is inside polygon if horisontal beam
        # to the right from point crosses polygon even number of times.
        # Works fine for non-convex polygons.
        # '''
        n = len(poly)
        inside = False

        p1x, p1y = poly[0]
        for i in range(1, n + 1):
            p2x, p2y = poly[i % n]
            if p1y == p2y:
                if y == p1y:
                    if min(p1x, p2x) <= x <= max(p1x, p2x):
                        # point is on horisontal edge
                        inside = include_edges
                        break
                    elif x < min(p1x, p2x):  # point is to the left from current edge
                        inside = not inside
            else:  # p1y!= p2y
                if min(p1y, p2y) <= y <= max(p1y, p2y):
                    xinters = (y - p1y) * (p2x - p1x) / float(p2y - p1y) + p1x

                    if x == xinters:  # point is right on the edge
                        inside = include_edges
                        break

                    if x < xinters:  # point is to the left from current edge
                        inside = not inside

            p1x, p1y = p2x, p2y

        return inside

    def plan_traj(self, insertion_position, insertion_normal, plane_normal, spline_points_array, planTrajectory, insertion_depth_per_pass, points_per_pass):
        # we want to move forward by 3mm along the orientation vector and then rotate about the entry point by 5 degrees per 27/35 mm of insertion
        # we stop rotating when we intersect with curve_central_fine (the spline through curve_central) or when we reach the end of maximum curvature for that insertion

        # to numpy array
        curve_central_fine = np.zeros(
            (spline_points_array.GetNumberOfPoints(), 3))
        for i in range(spline_points_array.GetNumberOfPoints()):
            spline_points_array.GetPoint(i, curve_central_fine[i, :])

        # make a rotation matrix that aligns the z axis with plane_normal and x axis with insertion_normal
        z = plane_normal
        x = insertion_normal
        y = np_cross(z, x)
        R = np.array([x, y, z]).T

        # make a 2d coordinate system with origin at insertion_position and x axis along insertion_normal
        x = insertion_normal
        y = np_cross(x, np.array([0, 0, 1]))
        R2 = np.array([x, y]).T
        # express curve_central_fine in this coordinate system
        curve_central_fine_2d = (
            R2.T @ (curve_central_fine - insertion_position).T).T

        # define disc as [(x1,y1),...,(xN,yN)] or [(x1,y1),...,(xN,yN),(x1,y1)] from curve_central_fine_2d
        disc = []
        for i in range(len(curve_central_fine_2d)):
            disc.append((curve_central_fine_2d[i, 0], curve_central_fine_2d[i, 1]))
        
        # make vtk polygon from curve_central_fine_2d
        # disc = vtk.vtkPolygon()
        # disc.GetPoints().SetNumberOfPoints(len(curve_central_fine_2d))
        # for i in range(len(curve_central_fine_2d)):
        #     disc.GetPoints().SetPoint(
        #         i, curve_central_fine_2d[i, 0], curve_central_fine_2d[i, 1], 0)
        # num_points = disc.GetPoints().GetNumberOfPoints()
        # disc_bounds = disc.GetPoints().GetBounds()
        # disc_normal = [0.0, 0.0, 1.0]
        # disc_points = disc.GetPoints()
        # disc = Path(curve_central_fine_2d)
        all_points = []
        for insertion_length in np.arange(insertion_depth_per_pass, 35.0, insertion_depth_per_pass):
            max_rot_angle_at_insertion = np.deg2rad(
                5) * 27/35 * insertion_length
            points, theta = self.find_points_on_arc(
                insertion_length, insertion_position, -max_rot_angle_at_insertion, max_rot_angle_at_insertion, R, num_points=points_per_pass)

            # find first point in points not in path
            in_disc = np.zeros(len(points), dtype=bool)
            for i in range(len(points)):
                point_in_plane = (R2.T @ (points[i] - insertion_position).T).T
                # append 0
                point_in_plane = np.hstack((point_in_plane, 0))
                # print("point_in_plane: " + str(point_in_plane))
                in_disc[i] = self.point_inside_polygon(point_in_plane[0], point_in_plane[1], disc)

            # if all of in_disc is false, then break (no more insertion)
            if not np.any(in_disc):
                break

            # if all of in_disc is true, then continue (keep inserting)
            if not np.all(in_disc):
                # if not all of in_disc is true, then find the first and last indices of in_disc that are true
                first_ind = np.argmax(in_disc)
                last_ind = len(in_disc) - np.argmax(in_disc[::-1]) - 1

                new_thetas = [theta[first_ind], theta[last_ind]]
                points, theta = self.find_points_on_arc(
                    insertion_length, insertion_position, np.min(new_thetas), np.max(new_thetas), R, num_points=points_per_pass)
            all_points.append(points)
        plan = []

        # add entry trajectory from insertion_position backwards along insertion_normal for 100mm with one point per mm
        n_entry_points = 100
        entry_points = np.zeros((n_entry_points, 3))
        for i in range(n_entry_points):
            entry_points[i, :] = insertion_position - insertion_normal * (n_entry_points - i)
        plan.append(entry_points)

        closest_idx_to_insertion = np.zeros(len(all_points), dtype=int)
        closest_idx_to_next = np.zeros(len(all_points), dtype=int)

        for i in range(len(all_points)):
            points = all_points[i]
            # closest point from a line through the insertion_position in the direction of the insertion_normal
            closest_idx_to_insertion[i] = np.argmin(np.linalg.norm(np.cross(
                points - insertion_position, insertion_normal), axis=1) / np.linalg.norm(insertion_normal))

        # find closest point in all_points[i] to all_points[i+1][closest_index_to_insertion[i+1]]
        for i in range(len(all_points)-1):
            points = all_points[i]
            next_points = all_points[i+1]
            closest_idx_to_next[i] = np.argmin(np.linalg.norm(
                points - next_points[closest_idx_to_insertion[i+1]], axis=1))

        # for each in all_points
        for i in range(len(all_points)):
            points = all_points[i]
            # add points from closest_idx_to_insertion[i] to idx=-1 to plan
            plan.append(points[closest_idx_to_insertion[i]::])
            # add points from idx=-1 to idx=0 to plan
            plan.append(points[-1:0:-1])
            # add points from idx=0 to closest_idx_to_next[i] to plan
            if i < len(all_points)-1:
                plan.append(points[0:closest_idx_to_next[i]+1])

        # loop through plan and stack all points into one array
        plan = np.vstack(plan)

        # remove all point from planTrajectory
        planTrajectory.RemoveAllControlPoints()
        # add all points from plan to planTrajectory
        pos = vtk.vtkVector3d()
        for i in range(len(plan)):
            # change to vtkvector3d
            pos.Set(plan[i, 0], plan[i, 1], plan[i, 2])
            planTrajectory.AddControlPointWorld(pos)
        # set planTrajectory to linear
        # planTrajectory.SetInterpolationTypeToLinear()

    def switch_cutplane(self, plane, curve):
        # make the axial view perpendicular to the plane
        # get the plane normal
        plane_normal = np.array((0.0, 0.0, 0.0))
        plane.GetNormal(plane_normal)
        # get the plane origin
        plane_origin = np.array((0.0, 0.0, 0.0))
        plane.GetOrigin(plane_origin)
        # get the axial view
        redSliceNode = slicer.util.getNode('vtkMRMLSliceNodeRed')

        # get point in the curve
        curve_point = np.array((0.0, 0.0, 0.0))
        points = curve.GetCurveWorld()
        points_array = points.GetPoints()
        points_array.GetPoint(0, curve_point)
        print("plane_origin is " + str(curve_point))
        setSlicePoseFromSliceNormalAndPosition(
            redSliceNode, plane_normal, curve_point)

    def switch_axial(self):
        # reset the axial view to be perpendicular to the RAS
        # get the axial view
        axial_view = slicer.app.layoutManager().sliceWidget(
            "Red").sliceLogic().GetSliceNode()
        # set the axial view normal to the RAS
        axial_view.SetOrientationToAxial()

    def addTestData(self, test_volume_data_filename, test_ct_directory):
        """ Load volume data  """

        print(test_volume_data_filename)
        print(test_ct_directory)
        _, label_volumeNode = slicer.util.loadLabelVolume(
            test_volume_data_filename, returnNode=True)

        # This is adapted from https://www.slicer.org/wiki/Documentation/4.3/Modules/VolumeRendering
        # The effect is that the volume rendering changes when the segmentation array changes
        slicer_logic = slicer.modules.volumerendering.logic()
        displayNode = slicer_logic.CreateVolumeRenderingDisplayNode()
        slicer.mrmlScene.AddNode(displayNode)
        displayNode.UnRegister(slicer_logic)
        slicer_logic.UpdateDisplayNodeFromVolumeNode(
            displayNode, label_volumeNode)
        label_volumeNode.AddAndObserveDisplayNodeID(displayNode.GetID())

        """ Load image data  """
        # noinspection PyUnresolvedReferences
        from DICOMLib import DICOMUtils
        with DICOMUtils.TemporaryDICOMDatabase() as db:
            DICOMUtils.importDicom(test_ct_directory, db)
            patientUID = db.patients()[0]
            DICOMUtils.loadPatientByUID(patientUID)

        # Resets focal point (pink wireframe bounding box) to center of scene
        layoutManager = slicer.app.layoutManager()
        threeDWidget = layoutManager.threeDWidget(0)
        threeDView = threeDWidget.threeDView()
        threeDView.resetFocalPoint()

    def toggleSliceIntersection(self):
        viewNodes = slicer.util.getNodesByClass('vtkMRMLSliceCompositeNode')
        for viewNode in viewNodes:
            viewNode.SetSliceIntersectionVisibility(
                int(not viewNode.GetSliceIntersectionVisibility()))

    def toggleSliceVisibility(self, name):
        layoutManager = slicer.app.layoutManager()
        for sliceViewName in layoutManager.sliceViewNames():
            if sliceViewName == name:
                controller = layoutManager.sliceWidget(
                    sliceViewName).sliceController()
                controller.setSliceVisible(
                    int(not controller.sliceLogic().GetSliceNode().GetSliceVisible()))

    def moveTargetToIntersectionButton(self, targetMarkupNode):
        crosshairNode = slicer.util.getNode('Crosshair')
        crosshairPos = crosshairNode.GetCrosshairRAS()
        targetMarkupNode.SetNthFiducialPosition(
            0, crosshairPos[0], crosshairPos[1], crosshairPos[2])

    def moveEntryToIntersectionButton(self, EntryMarkupNode):
        crosshairNode = slicer.util.getNode('Crosshair')
        crosshairPos = crosshairNode.GetCrosshairRAS()
        EntryMarkupNode.SetNthFiducialPosition(
            0, crosshairPos[0], crosshairPos[1], crosshairPos[2])

    def jumpToMarkup(self, MarkupNode):
        pos = [0.0, 0.0, 0.0]
        MarkupNode.GetNthFiducialPosition(0, pos)

        for name in ['Red', 'Yellow', 'Green']:
            sliceNode = slicer.app.layoutManager().sliceWidget(name).mrmlSliceNode()
            sliceNode.JumpSlice(pos[0], pos[1], pos[2])

    def alignAxesWithTrajectory(self, targetMarkupNode, EntryMarkupNode):

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
        setSlicePoseFromSliceNormalAndPosition(
            redSliceNode, sliceNormal, slicePosition)

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

    def resetAxesToASC(self, targetMarkupNode):
        redSliceNode = slicer.util.getNode('vtkMRMLSliceNodeRed')
        yellowSliceNode = slicer.util.getNode('vtkMRMLSliceNodeYellow')
        greenSliceNode = slicer.util.getNode('vtkMRMLSliceNodeGreen')
        redSliceNode.SetOrientationToAxial()
        yellowSliceNode.SetOrientationToSagittal()
        greenSliceNode.SetOrientationToCoronal()
        p_target = np.array([0.0, 0.0, 0.0])
        targetMarkupNode.GetNthFiducialPosition(0, p_target)


# noinspection PyMethodMayBeStatic
class CM_PathTest(ScriptedLoadableModuleTest):
    """
  This is the test case for your scripted module.
  Uses ScriptedLoadableModuleTest base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """

    def setUp(self):
        slicer.mrmlScene.Clear(0)

    def runTest(self):
        self.setUp()
        self.test_CM_Path1()

    def test_CM_Path1(self):
        self.delayDisplay("Starting the test:")
        self.delayDisplay("No tests for now!")
        self.delayDisplay('Test passed!')
