import vtk
import numpy as np
from numpy import random
import pyrealsense2 as rs
import time

IMG_H = 480
IMG_W = 640
MAX_DIST = 5000.0
DIST_NORMALIZE = 1000.0

class RealsenseViewer:
    def __init__(self, mode):
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

    def init_realsense(self):
        self.pipeline = rs.pipeline()
        self.rs_config = rs.config()
        if self.mode_depth or self.mode_register:
            self.rs_config.enable_stream(
                rs.stream.depth,
                IMG_W,
                IMG_H,
                rs.format.z16,
                30
            )
        if self.mode_color or self.mode_register:
            self.rs_config.enable_stream(
                rs.stream.color,
                IMG_W,
                IMG_H,
                rs.format.bgr8,
                30
            )
        self.pipeline.start(self.rs_config)

    def init_vtk(self):
        self.renderWindow = vtk.vtkRenderWindow()
        numOfViewPorts = self.mode_color + self.mode_register + self.mode_depth
        if numOfViewPorts == 2:
            self.renderWindow.SetSize(1000, 500)
        else:
            self.renderWindow.SetSize(1000, 1000)

        if self.mode_depth or self.mode_register:
            self.vtkPoints = vtk.vtkPoints()
            self.vtkCells = vtk.vtkCellArray()
            self.vtkDepth = vtk.vtkDoubleArray()
            self.vtkDepth.SetName('DepthArray')

        # if self.mode_color or self.mode_register:
            # self.vtk_ColorData = vtk.vtkUnsignedCharArray()
            # self.vtk_ColorData.SetNumberOfComponents(3)
            # self.vtk_ColorData.SetName('Colors')

        if self.mode_color:
            self.init_vtk_2DImage()
        if self.mode_depth:
            self.init_vtk_PointCloud()
        if self.mode_register:
            self.init_vtk_ColorPointCloud()

        self.clearPoints()

        if numOfViewPorts == 3:
            self.image_renderer.SetViewport(0, 0, 0.5, 0.5)
            self.rgb_renderer.SetViewport(0.5, 0, 1, 0.5)
            self.pc_renderer.SetViewport(0, 0.5, 0.5, 1)
            camera = self.rgb_renderer.GetActiveCamera()
            self.pc_renderer.SetActiveCamera(camera)
        elif numOfViewPorts == 2:
            if not self.mode_color:
                self.rgb_renderer.SetViewport(0, 0, 0.5, 1)
                self.pc_renderer.SetViewport(0.5, 0, 1, 1)
                camera = self.rgb_renderer.GetActiveCamera()
                self.pc_renderer.SetActiveCamera(camera)
            elif not self.mode_register:
                self.image_renderer.SetViewport(0, 0, 0.5, 1)
                self.pc_renderer.SetViewport(0.5, 0, 1, 1)
            else:
                self.rgb_renderer.SetViewport(0, 0, 0.5, 1)
                self.image_renderer.SetViewport(0.5, 0, 1, 1)

        self.renderWindowInteractor = vtk.vtkRenderWindowInteractor()
        self.renderWindowInteractor.SetRenderWindow(self.renderWindow)
        self.renderWindowInteractor.Initialize()

        # if self.mode_color:
            # self.renderWindowInteractor.SetInteractorStyle(
                # vtk.vtkInteractorStyleImage()
            # )

        self.init_axes()

        self.camera_reset = False
        timerId = self.renderWindowInteractor.CreateRepeatingTimer(10)
        self.renderWindowInteractor.AddObserver('TimerEvent',
            self.update
        )

        # if self.mode_color:
            # self.image_renderer.ResetCamera()
        # if self.mode_register:
            # self.rgb_renderer.ResetCamera()
        # if self.mode_depth:
            # self.pc_renderer.ResetCamera()

        self.renderWindowInteractor.Start()

    # def init_vtk_2DImage(self):
        # self.image_vtkImageData = vtk.vtkImageData()
        # self.image_vtkImageData.SetDimensions(IMG_H, IMG_W, 1)
        # self.image_vtkImageData.AllocateScalars(vtk.VTK_DOUBLE, 3)

        # self.image_actor = vtk.vtkImageActor()
        # self.image_actor.SetInputData(self.image_vtkImageData)
        # self.image_actor.SetOrientation(0, 0, -90)

        # self.image_renderer = vtk.vtkRenderer()
        # self.image_renderer.SetBackground(.2, .3, .4)
        # self.image_renderer.AddActor2D(self.image_actor)
        # self.image_renderer.ResetCamera()

        # self.renderWindow.AddRenderer(self.image_renderer)

    def init_vtk_2DImage(self):
        self.vtk_static_points = vtk.vtkPoints()
        self.vtk_static_cells = vtk.vtkCellArray()
        self.vtk_static_depth = vtk.vtkDoubleArray()
        self.vtk_static_depth.SetName('DepthArray')

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
        self.image_renderer.SetBackground(.2, .3, .4)
        self.image_renderer.AddActor(self.image_actor)
        self.image_renderer.ResetCamera()

        self.renderWindow.AddRenderer(self.image_renderer)

    def init_vtk_PointCloud(self):
        self.vtk_plData = vtk.vtkPolyData()
        # self.vtk_plData.SetPoints(self.vtkPoints)
        # self.vtk_plData.SetVerts(self.vtkCells)
        # self.vtk_plData.GetPointData().SetScalars(self.vtkDepth)
        # self.vtk_plData.GetPointData().SetActiveScalars('DepthArray')

        pc_mapper = vtk.vtkPolyDataMapper()
        pc_mapper.SetColorModeToDefault()
        pc_mapper.SetScalarRange(.0, 255.0)
        pc_mapper.SetScalarVisibility(1)
        pc_mapper.SetInputData(self.vtk_plData)
        self.pc_vtkActor = vtk.vtkActor()
        self.pc_vtkActor.SetMapper(pc_mapper)

        self.pc_renderer = vtk.vtkRenderer()
        self.pc_renderer.SetBackground(.2, .3, .4)
        self.pc_renderer.AddActor(self.pc_vtkActor)
        self.pc_renderer.ResetCamera()

        self.renderWindow.AddRenderer(self.pc_renderer)

    def init_vtk_ColorPointCloud(self):
        # self.vtk_ColorData = vtk.vtkUnsignedCharArray()
        # self.vtk_ColorData.SetNumberOfComponents(3)
        # self.vtk_ColorData.SetName('Colors')

        self.rgb_vtkPolyData = vtk.vtkPolyData()
        # self.rgb_vtkPolyData.SetPoints(self.vtkPoints)
        # self.rgb_vtkPolyData.SetVerts(self.vtkCells)
        # self.rgb_vtkPolyData.GetPointData().SetScalars(
            # self.vtk_ColorData
        # )
        # self.rgb_vtkPolyData.GetPointData().SetActiveScalars('Colors')

        rgb_mapper = vtk.vtkPolyDataMapper()
        rgb_mapper.SetScalarVisibility(1)
        rgb_mapper.SetInputData(self.rgb_vtkPolyData)

        self.rgb_vtkActor = vtk.vtkActor()
        self.rgb_vtkActor.SetMapper(rgb_mapper)

        self.rgb_renderer = vtk.vtkRenderer()
        self.rgb_renderer.SetBackground(.2, .3, .4)
        self.rgb_renderer.AddActor(self.rgb_vtkActor)
        self.rgb_renderer.ResetCamera()

        self.renderWindow.AddRenderer(self.rgb_renderer)

    def init_axes(self):
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(50, 50, 50)
        widget = vtk.vtkOrientationMarkerWidget()
        widget.SetOutlineColor(0.9300, 0.5700, 0.1300)
        widget.SetOrientationMarker(axes)
        widget.SetInteractor(self.renderWindowInteractor)
        widget.SetViewport(0.0, 0.0, 0.2, 0.2)
        widget.SetEnabled(1)
        widget.InteractiveOn()

    def clearPoints(self):
        if self.mode_register or self.mode_depth:
            self.vtkPoints = vtk.vtkPoints()
            self.vtkCells = vtk.vtkCellArray()
            self.vtkDepth = vtk.vtkDoubleArray()
            self.vtkDepth.SetName('DepthArray')

        if self.mode_depth:
            self.vtk_plData.SetPoints(self.vtkPoints)
            self.vtk_plData.SetVerts(self.vtkCells)
            self.vtk_plData.GetPointData().SetScalars(self.vtkDepth)
            self.vtk_plData.GetPointData().SetActiveScalars('DepthArray')

        if self.mode_register or self.mode_color:
            self.vtk_ColorData = vtk.vtkUnsignedCharArray()
            self.vtk_ColorData.SetNumberOfComponents(3)
            self.vtk_ColorData.SetName('Colors')

        if self.mode_register:
            self.rgb_vtkPolyData.SetPoints(self.vtkPoints)
            self.rgb_vtkPolyData.SetVerts(self.vtkCells)
            self.rgb_vtkPolyData.GetPointData().SetScalars(
                self.vtk_ColorData
            )
            self.rgb_vtkPolyData.GetPointData().SetActiveScalars('Colors')

        if self.mode_color:
            self.vtk_imData.SetPoints(self.vtk_static_points)
            self.vtk_imData.SetVerts(self.vtk_static_cells)
            self.vtk_imData.GetPointData().SetScalars(
                self.vtk_ColorData
            )
            self.vtk_imData.GetPointData().SetActiveScalars('Colors')

    def addDepthPoint(self, point):
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
        i = point[0]
        j = point[1]
        bgr = point[2]
        # if self.mode_color:
            # self.image_vtkImageData.SetScalarComponentFromDouble(
                # i, j, 0, 0, bgr[2]
            # )
            # self.image_vtkImageData.SetScalarComponentFromDouble(
                # i, j, 0, 1, bgr[1]
            # )
            # self.image_vtkImageData.SetScalarComponentFromDouble(
                # i, j, 0, 2, bgr[0]
            # )
        if self.mode_register or self.mode_color:
            self.vtk_ColorData.InsertNextTuple3(bgr[2], bgr[1], bgr[0])

    def update(self, obj=None, event=None):
        frame = self.pipeline.wait_for_frames()
        self.clearPoints()

        if self.mode_color or self.mode_register:
            color_frame = frame.get_color_frame()
            if not color_frame:
                print('Warning: missing frame')
                return
            color_data = np.asanyarray(color_frame.get_data())

        if self.mode_depth or self.mode_register:
            depth_frame = frame.get_depth_frame()
            if not depth_frame:
                print('Warning: missing frame')
                return
            depth_image = np.asanyarray(depth_frame.get_data())

        for i in range(IMG_H):
            for j in range(IMG_W):
                if i % 3 or j % 3:
                    continue
                if self.mode_color or self.mode_register:
                    bgr = color_data[i, j]
                    self.addColorPoint([i, j, bgr])

                if self.mode_depth or self.mode_register:
                    dist = min(MAX_DIST, depth_image[i, j]) / MAX_DIST * \
                            DIST_NORMALIZE
                    # self.addDepthPoint([i, j, dist * dist * dist * 500])
                    self.addDepthPoint([i, j, dist])

        if not self.camera_reset:
            self.camera_reset = True
            if self.mode_depth:
                self.vtk_plData.Modified()
                self.pc_renderer.ResetCamera()

            if self.mode_register:
                self.rgb_vtkPolyData.Modified()
                self.rgb_renderer.ResetCamera()

            if self.mode_color:
                # self.image_vtkImageData.Modified()
                self.vtk_imData.Modified()
                self.image_renderer.ResetCamera()

        # self.renderWindow.Render()
        self.renderWindowInteractor.Render()

viewer = RealsenseViewer({'color', 'depth', 'register'})
# viewer = RealsenseViewer({'depth', 'register'})
