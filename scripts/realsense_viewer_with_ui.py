import vtk
import numpy as np
from numpy import random
import pyrealsense2 as rs
import time
from PyQt5 import QtCore, QtGui, QtWidgets
import sys
import vtk
from PyQt5 import uic
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import *
from PyQt5 import Qt
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from time import gmtime, strftime
from ui_layout import Ui_MainWindow

IMG_H = 480     # Image height
IMG_W = 640     # Image width
MAX_DIST = 5000.0   # Max cut-off distance
DIST_NORMALIZE = 1000.0 # Normalize distance to this range

class RealsenseViewer:
    def __init__(self, mode):
        """
        Mode: contains the following mode toggles
            depth: Turn on depth image with default color map
            color: Turn on 2D rgb image
            register: Turn on registered depth image (depth image with rgb)
        """
        #QtGui.QMainWindow.__init__(self,parent)
        app = QtWidgets.QApplication(sys.argv)
        MainWindow = QtWidgets.QMainWindow()
       # MainWindow = QtWidgets.QMainWindow()
        self.ui=Ui_MainWindow()
        self.ui.setupUi(MainWindow)
        self.ui.vtkWidget = QVTKRenderWindowInteractor(self.ui.widget)
      #  self.ui.vtkWidget.SetSize(500,300)
        self.mode = mode
        self.mode_depth = False
        self.mode_color = False
        self.mode_register = False
        if 'depth' in self.mode:
            self.mode_depth = True
        if 'color' in self.mode:
            self.mode_color = True
        if 'register' in self.mode:
            self.mode_register = True
        if not (self.mode_depth or self.mode_color or self.mode_register):
            raise ValueError(
                'Mode must contain at least one of \'color\', \'depth\' \
                or \'register\'.'
            )

        self.init_realsense()
        self.init_vtk()
       
        self.ui.Snapshot.clicked.connect(lambda: self.snapshot(self.renderWindow))
        self.ui.Roll.clicked.connect(lambda:self.roll(self.rgb_renderer,self.renderWindow))
        self.ui.Azimuth.clicked.connect(lambda:self.azimuth(self.rgb_renderer,self.renderWindow))
        self.ui.Pitch.clicked.connect(lambda:self.pitch(self.rgb_renderer,self.renderWindow))
        self.ui.reset.clicked.connect(lambda:self.reset(self.rgb_renderer,self.renderWindow,self.rgb_vtkActor))
        #self.ui.reset.clicked.connect(lambda:self.reset(self.pc_renderer,self.renderWindow,self.pc_vtkActor))
       # MainWindow.show()
       
        MainWindow.show()
        sys.exit(app.exec_())


    def init_realsense(self):
        self.pipeline = rs.pipeline()
        self.rs_config = rs.config()
        if self.mode_depth or self.mode_register:
            # Turn on depth image channel on Realsense camera
            self.rs_config.enable_stream(
                rs.stream.depth,
                IMG_W,
                IMG_H,
                rs.format.z16,
                30
            )
        if self.mode_color or self.mode_register:
            # Turn on RGB channel on Realsense camera
            self.rs_config.enable_stream(
                rs.stream.color,
                IMG_W,
                IMG_H,
                rs.format.bgr8,
                30
            )
        self.pipeline.start(self.rs_config)

    def init_vtk(self):
        """
        Initialize all VTK related settings
        """
        self.renderWindow = self.ui.vtkWidget.GetRenderWindow()
        self.renderWindow.SetSize(600,500)
        numOfViewPorts = self.mode_color + self.mode_register + self.mode_depth
        #if numOfViewPorts == 2:
        #    self.renderWindow.SetSize(1000, 500)
        #else:
        #    self.renderWindow.SetSize(1000, 1000)
        #self.renderWindow.SetSize(200,200)
        # Initialize Pointclouds containers for depth and register modes
        if self.mode_depth or self.mode_register:
            self.vtkPoints = vtk.vtkPoints()
            self.vtkCells = vtk.vtkCellArray()
            self.vtkDepth = vtk.vtkDoubleArray()
            self.vtkDepth.SetName('DepthArray')

        # Initialize all rendered views
        if self.mode_color:
            self.init_vtk_2DImage()
        if self.mode_depth:
            self.init_vtk_PointCloud()
        if self.mode_register:
            self.init_vtk_ColorPointCloud()

        # Initialize Pointcloud data containers
        self.clearPoints()

        # Defines the layout of all rendered views
        if numOfViewPorts == 3:
            self.pc_renderer.SetViewport(0, 0, 0.5, 2/3)
            self.rgb_renderer.SetViewport(0.5, 0, 1, 2/3)
            self.image_renderer.SetViewport(0, 2/3, 1, 1)

            # Camera synchronization
            camera = self.rgb_renderer.GetActiveCamera()
            #camera.Zoom(2)
            self.pc_renderer.SetActiveCamera(camera)

            #self.image_renderer.SetActiveCamera(camera)
        elif numOfViewPorts == 2:
            if not self.mode_color:
                self.rgb_renderer.SetViewport(0, 0, 0.5, 1)
                self.pc_renderer.SetViewport(0.5, 0, 1, 1)

                # Camera synchronization
                camera = self.rgb_renderer.GetActiveCamera()
                self.pc_renderer.SetActiveCamera(camera)
            elif not self.mode_register:
                self.image_renderer.SetViewport(0, 0, 0.5, 1)
                self.pc_renderer.SetViewport(0.5, 0, 1, 1)
            else:
                self.rgb_renderer.SetViewport(0, 0, 0.5, 1)
                self.image_renderer.SetViewport(0.5, 0, 1, 1)

        #self.renderWindowInteractor = vtk.vtkRenderWindowInteractor()
        #self.renderWindowInteractor.SetRenderWindow(self.renderWindow)
        self.iren=self.ui.vtkWidget.GetRenderWindow().GetInteractor()
        #self.iren.UpdateSize(500,400)
        self.iren.Initialize()
        #print( self.iren)

        # This will disable rotation for all rendered views
        # self.renderWindowInteractor.SetInteractorStyle(
            # vtk.vtkInteractorStyleImage()
        # )

        # self.init_axes() # this is not working :(

        self.camera_reset = False # this will control re-setting all cameras
                                  # during the first iteration

        # Create callback function for updating
        timerId = self.iren.CreateRepeatingTimer(10) # 10ms
        self.iren.AddObserver('TimerEvent',
            self.update
        )

        # Program start
        self.iren.Start()

    def init_vtk_2DImage(self):
        """
        Viewport Initialization: 2D Image Mode
        """
        # These are the static pointcloud containers for the 2D image, all
        # depth is set to a constant value 0 to display a flat image plane
        self.vtk_static_points = vtk.vtkPoints()
        self.vtk_static_cells = vtk.vtkCellArray()
        self.vtk_static_depth = vtk.vtkDoubleArray()
        self.vtk_static_depth.SetName('DepthArray')

        # Setting all values to 0
        for i in range(IMG_H):
            for j in range(IMG_W):
                if i % 3 or j % 3:
                    continue
                pointId = self.vtk_static_points.InsertNextPoint(
                    [640 - j, 480 - i, 0]
                )
                self.vtk_static_depth.InsertNextValue(0)
                self.vtk_static_cells.InsertNextCell(1)
                self.vtk_static_cells.InsertCellPoint(pointId)
        self.vtk_static_cells.Modified()
        self.vtk_static_points.Modified()
        self.vtk_static_depth.Modified()

        self.vtk_imData = vtk.vtkPolyData()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetScalarVisibility(1)
        mapper.SetInputData(self.vtk_imData)

        self.image_actor = vtk.vtkActor()
        self.image_actor.SetMapper(mapper)
        
        self.image_renderer = vtk.vtkRenderer()
        self.image_renderer.SetBackground(0, 0, 0)
        self.image_renderer.AddActor(self.image_actor)
        self.image_renderer.ResetCamera()
        #self.image_renderer.GetActiveCamera().Zoom(2)

        self.renderWindow.AddRenderer(self.image_renderer)

    def init_vtk_PointCloud(self):
        """
        Viewport Initialization: Depth Mode
        """
        self.vtk_plData = vtk.vtkPolyData()

        pc_mapper = vtk.vtkPolyDataMapper()
        pc_mapper.SetColorModeToDefault()
        pc_mapper.SetScalarRange(.0, 255.0)
        pc_mapper.SetScalarVisibility(1)
        pc_mapper.SetInputData(self.vtk_plData)
        self.pc_vtkActor = vtk.vtkActor()
        self.pc_vtkActor.SetMapper(pc_mapper)

        self.pc_renderer = vtk.vtkRenderer()
        self.pc_renderer.SetBackground(0, 0, 0)
        self.pc_renderer.AddActor(self.pc_vtkActor)
        self.pc_renderer.ResetCamera()
        self.pc_renderer.GetActiveCamera().Zoom(2)

        self.renderWindow.AddRenderer(self.pc_renderer)

    def init_vtk_ColorPointCloud(self):
        """
        Viewport Initialization: Registered Depth Image Mode
        """
        self.rgb_vtkPolyData = vtk.vtkPolyData()

        rgb_mapper = vtk.vtkPolyDataMapper()
        rgb_mapper.SetScalarVisibility(1)
        rgb_mapper.SetInputData(self.rgb_vtkPolyData)

        self.rgb_vtkActor = vtk.vtkActor()
        self.rgb_vtkActor.SetMapper(rgb_mapper)

        self.rgb_renderer = vtk.vtkRenderer()
        self.rgb_renderer.SetBackground(0, 0, 0)
        self.rgb_renderer.AddActor(self.rgb_vtkActor)
        self.rgb_renderer.ResetCamera()
        self.rgb_renderer.GetActiveCamera().Zoom(2)

        self.renderWindow.AddRenderer(self.rgb_renderer)

    def init_axes(self):
        """
        Initialize a small axes widget
        """
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(50, 50, 50)
        widget = vtk.vtkOrientationMarkerWidget()
        widget.SetOutlineColor(0.9300, 0.5700, 0.1300)
        widget.SetOrientationMarker(axes)
        widget.SetInteractor(self.renderWindowInteractor)
        widget.SetInteractor(self.renderWindowInteractor)
        widget.SetViewport(0.0, 0.0, 0.2, 0.2)
        widget.SetEnabled(1)
        # widget.InteractiveOn() # disable interaction for axes widget

    def clearPoints(self):
        """
        Clear all containers to allow update new data, this function needs to
        be called during the initialization of the program and everytime you
        want to update a new frame.
        """
        # Clear Pointcloud containers for registered mode and depth mode
        if self.mode_register or self.mode_depth:
            self.vtkPoints = vtk.vtkPoints()
            self.vtkCells = vtk.vtkCellArray()
            self.vtkDepth = vtk.vtkDoubleArray()
            self.vtkDepth.SetName('DepthArray')

        # Clear the pointcloud data container for depth mode
        if self.mode_depth:
            self.vtk_plData.SetPoints(self.vtkPoints)
            self.vtk_plData.SetVerts(self.vtkCells)
            self.vtk_plData.GetPointData().SetScalars(self.vtkDepth)
            self.vtk_plData.GetPointData().SetActiveScalars('DepthArray')

        # Clear color container for registered and 2d image mode
        if self.mode_register or self.mode_color:
            self.vtk_ColorData = vtk.vtkUnsignedCharArray()
            self.vtk_ColorData.SetNumberOfComponents(3)
            self.vtk_ColorData.SetName('Colors')

        # Clear the pointcloud data container for registered mode
        if self.mode_register:
            self.rgb_vtkPolyData.SetPoints(self.vtkPoints)
            self.rgb_vtkPolyData.SetVerts(self.vtkCells)
            self.rgb_vtkPolyData.GetPointData().SetScalars(
                self.vtk_ColorData
            )
            self.rgb_vtkPolyData.GetPointData().SetActiveScalars('Colors')

        # Clear the color container for 2d image mode
        if self.mode_color:
            self.vtk_imData.SetPoints(self.vtk_static_points)
            self.vtk_imData.SetVerts(self.vtk_static_cells)
            self.vtk_imData.GetPointData().SetScalars(
                self.vtk_ColorData
            )
            self.vtk_imData.GetPointData().SetActiveScalars('Colors')

    def addDepthPoint(self, point):
        """
        Add one depth pixel to the pointcloud data containers
        """
        pointId = self.vtkPoints.InsertNextPoint(
            [640 - point[1], 480 - point[0], 0 - point[2]]
            # [point[0], point[1], point[2]]
        )
        self.vtkDepth.InsertNextValue(point[2])
        self.vtkCells.InsertNextCell(1)
        self.vtkCells.InsertCellPoint(pointId)

        self.vtkCells.Modified()
        self.vtkPoints.Modified()
        self.vtkDepth.Modified()

    def addColorPoint(self, point):
        """
        Add one rgb pixel to the color data containers (note that here the
        data is in BGR format)
        """
        # i = point[0]
        # j = point[1]
        bgr = point[2]
        if self.mode_register or self.mode_color:
            self.vtk_ColorData.InsertNextTuple3(bgr[2], bgr[1], bgr[0])
    def snapshot(self,renWin):
        w2if = vtk.vtkWindowToImageFilter()
        w2if.SetInput(renWin)
        w2if.SetScale(1)
        w2if.ReadFrontBufferOff()
        w2if.Update()
       
       
        writer = vtk.vtkJPEGWriter()
        filename= strftime("%Y-%m-%d_%H-%M-%S", gmtime())+".jpg"
        writer.SetFileName(filename)
        writer.SetInputConnection(w2if.GetOutputPort())
        writer.Write()
    def roll(self,ren,renWin):
        temp_cam=ren.GetActiveCamera()
        #temp_cam.SetObliqueAngles(30,60)
       # temp_cam.Roll(30)
        temp_cam.Roll(10)
        ren.SetActiveCamera(temp_cam)
        renWin.Render()
    def azimuth(self,ren,renWin):
        temp_cam=ren.GetActiveCamera()
        #temp_cam.SetObliqueAngles(30,60)
       # temp_cam.Roll(30)
        temp_cam.Azimuth(5)
        ren.SetActiveCamera(temp_cam)
        renWin.Render()
    def pitch(self,ren,renWin):
        temp_cam=ren.GetActiveCamera()
        #temp_cam.SetObliqueAngles(30,60)
       # temp_cam.Roll(30)
        temp_cam.Pitch(1)
        ren.SetActiveCamera(temp_cam)
        renWin.Render()
    def reset(self,ren,renWin,actor):
        #actor.GetProperty().SetPosition(0,0,0)
        #actor.SetPosition(0,0,0)
        #actor.SetOrientation(0,0,0)

        #source.SetCenter(0, 0, 0)
        #temp_cam=ren.GetActiveCamera()
        #print(temp_cam)
        #temp_cam.SetViewUp(0,1,0)
        #temp_cam.SetObliqueAngles(45,90)
        #ren.SetActiveCamera(temp_cam)
        #ren.ResetCamera()
        #print(ren.GetActiveCamera())
        ren.GetActiveCamera().SetObliqueAngles(45,90)
        #ren.GetActiveCamera().Roll(30)
        #ren.GetActiveCamera().Zoom(1.5)
        #ren.GetActiveCamera().SetPosition(0
        ren.GetActiveCamera().SetViewUp(0,1,0)
        ren.GetActiveCamera().SetRoll(0)

        ren.ResetCamera()
        ren.GetActiveCamera().Zoom(2)
        #ren.AddActor(actor)
   
        
        renWin.Render()

    def update(self, obj=None, event=None):
        """
        The main update function, will be called every 10ms (approximately)
        from the renderWindowInteractor
        """
        frame = self.pipeline.wait_for_frames()
        self.clearPoints()
        self.renderWindow.SetSize(600,500)
        # Get a RGB color frame
        if self.mode_color or self.mode_register:
            color_frame = frame.get_color_frame()
            if not color_frame:
                print('Warning: missing frame')
                return
            color_data = np.asanyarray(color_frame.get_data())

        # Get a Depth frame
        if self.mode_depth or self.mode_register:
            depth_frame = frame.get_depth_frame()
            if not depth_frame:
                print('Warning: missing frame')
                return
            depth_image = np.asanyarray(depth_frame.get_data())

        # Update every pixel for all data containers
        for i in range(IMG_H):
            for j in range(IMG_W):
                if i % 3 or j % 3:
                    continue
                if self.mode_color or self.mode_register:
                    bgr = color_data[i, j]
                    self.addColorPoint([i, j, bgr])

                if self.mode_depth or self.mode_register:
                    # Cut-off the value into the range of [0, MAX_DIST], and
                    # then normalize to [0, DIST_NORMALIZE]
                    dist = min(MAX_DIST, depth_image[i, j]) / MAX_DIST * \
                            DIST_NORMALIZE
                    self.addDepthPoint([i, j, dist])

        # Indicate the corresponding data container are updated, but I don't
        # know if this will have any effect
        if self.mode_depth:
            self.vtk_plData.Modified()
        if self.mode_register:
            self.rgb_vtkPolyData.Modified()
        if self.mode_color:
            self.vtk_imData.Modified()

        # Refresh the camera in the first frame, otherwise you won't able to
        # interact with the cameras
        if not self.camera_reset:
            self.camera_reset = True
            if self.mode_depth:
                self.pc_renderer.ResetCamera()
                self.pc_renderer.GetActiveCamera().Zoom(2)
            if self.mode_register:
                self.rgb_renderer.ResetCamera()
                self.rgb_renderer.GetActiveCamera().Zoom(2)
            if self.mode_color:
                self.image_renderer.ResetCamera()

        # self.renderWindow.Render()

        self.iren.Render()

viewer = RealsenseViewer({'color', 'depth', 'register'})
# viewer = RealsenseViewer({'depth', 'register'})
# viewer = RealsenseViewer({'depth'})