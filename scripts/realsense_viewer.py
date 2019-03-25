import vtk
import numpy as np
from numpy import random
import pyrealsense2 as rs
import threading
import time


class VtkPointCloud:
    def __init__(self, mode=0, zMin=0.0, zMax=255.0, maxNumPoints=1e10):
        """
        :param mode: 0 - Stereo Mode
                     1 - RGB Mode
                     2 - Both
        """
        self.mode = mode
        self.maxNumPoints = maxNumPoints
        self.stereo_vtkPolyData = vtk.vtkPolyData()
        self.rgb_vtkPolyData = vtk.vtkPolyData()
        stereo_mapper = vtk.vtkPolyDataMapper()
        rgb_mapper = vtk.vtkPolyDataMapper()

        if self.mode not in {0, 1, 2}:
            raise ValueError('Invalid Mode. Choose from 0 (Stereo Mode), 1 (RGB Mode), 2 (Both)')
        if self.mode in {0, 2}:  # Stereo
            stereo_mapper.SetColorModeToDefault()
            stereo_mapper.SetScalarRange(zMin, zMax)
            stereo_mapper.SetScalarVisibility(1)
            stereo_mapper.SetInputData(self.stereo_vtkPolyData)
            self.stereo_vtkActor = vtk.vtkActor()
            self.stereo_vtkActor.SetMapper(stereo_mapper)
        if self.mode in {1, 2}:  # RGB
            rgb_mapper.SetScalarVisibility(1)
            rgb_mapper.SetInputData(self.rgb_vtkPolyData)
            self.rgb_vtkActor = vtk.vtkActor()
            self.rgb_vtkActor.SetMapper(rgb_mapper)

        self.numPoints = 0  # For debugging
        self.clearPoints()

    def addPoint(self, point):
        pointId = self.vtkPoints.InsertNextPoint([640 - point[0], 480 - point[1], 0 - point[2]])
        self.vtkDepth.InsertNextValue(point[2])
        self.vtkCells.InsertNextCell(1)
        self.vtkCells.InsertCellPoint(pointId)
        self.vtkCells.Modified()
        self.vtkPoints.Modified()
        self.vtkDepth.Modified()
        self.stereo_vtkPolyData.Modified()
        self.rgb_vtkPolyData.Modified()

    def clearPoints(self):
        self.vtkPoints = vtk.vtkPoints()
        self.vtkCells = vtk.vtkCellArray()
        self.vtkDepth = vtk.vtkDoubleArray()
        self.vtkDepth.SetName('DepthArray')
        self.stereo_vtkPolyData.SetPoints(self.vtkPoints)
        self.stereo_vtkPolyData.SetVerts(self.vtkCells)
        self.rgb_vtkPolyData.SetPoints(self.vtkPoints)
        self.rgb_vtkPolyData.SetVerts(self.vtkCells)

        if self.mode in {1, 2}:  # RGB
            self.Colors = vtk.vtkUnsignedCharArray()
            self.Colors.SetNumberOfComponents(3)
            self.Colors.SetName("Colors")
            self.rgb_vtkPolyData.GetPointData().SetScalars(self.Colors)
            self.rgb_vtkPolyData.GetPointData().SetActiveScalars('Colors')
        if self.mode in {0, 2}:  # Stereo
            self.stereo_vtkPolyData.GetPointData().SetScalars(self.vtkDepth)
            self.stereo_vtkPolyData.GetPointData().SetActiveScalars('DepthArray')

    def update_pointcloud(self, threadLock, update_on):
        while update_on.is_set():
            time.sleep(0.01)
            threadLock.acquire()
            self.clearPoints()
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            color_data = np.asanyarray(color_frame.get_data())
            if not depth_frame or not color_frame:
                continue
            for y in range(480):
                for x in range(640):
                    if x % 3 or y % 3:
                        continue
                    dist = depth_frame.get_distance(x, y) * 50
                    self.addPoint([x, y, dist])
                    if self.mode in {1, 2}:  # RGB
                        rgb = color_data[y, x]
                        self.Colors.InsertNextTuple3(rgb[0], rgb[1], rgb[2])
            threadLock.release()

    def update(self, threadLock, update_on):
        thread = threading.Thread(target=self.update_pointcloud, args=(threadLock, update_on))
        thread.start()


class Visualization:
    def __init__(self, threadLock, pointCloud, iterations):
        self.threadLock = threadLock
        self.iterations = iterations
        self.pointCloud = pointCloud
        self.mode = pointCloud.mode
        self.renderWindow = vtk.vtkRenderWindow()
        self.renderWindow.SetSize(1000, 800)

        if self.mode in {0, 2}:  # Stereo
            self.stereo_renderer = vtk.vtkRenderer()
            self.stereo_renderer.SetBackground(.2, .3, .4)
            self.stereo_renderer.ResetCamera()
            self.stereo_renderer.AddActor(pointCloud.stereo_vtkActor)
            self.renderWindow.AddRenderer(self.stereo_renderer)

        if self.mode in {1, 2}:  # RGB
            self.rgb_renderer = vtk.vtkRenderer()
            self.rgb_renderer.SetBackground(.2, .3, .4)
            self.rgb_renderer.ResetCamera()
            self.rgb_renderer.AddActor(pointCloud.rgb_vtkActor)
            self.renderWindow.AddRenderer(self.rgb_renderer)

        if self.mode == 2:  # Both
            self.stereo_renderer.SetViewport(0, 0, 0.5, 1)
            self.rgb_renderer.SetViewport(0.5, 0, 1, 1)

        # Interactor
        self.renderWindowInteractor = vtk.vtkRenderWindowInteractor()
        self.renderWindowInteractor.SetRenderWindow(self.renderWindow)
        self.renderWindowInteractor.Initialize()

        self.renderWindowInteractor.AddObserver('TimerEvent', self.update_visualization)
        timerId = self.renderWindowInteractor.CreateRepeatingTimer(30)

    def update_visualization(self, obj=None, event=None):
        time.sleep(0.01)
        self.threadLock.acquire()
        if self.mode in {0, 2}:  # Stereo
            self.stereo_renderer.GetRenderWindow().Render()
            if self.iterations == 30:
                self.stereo_renderer.ResetCamera()
        if self.mode in {1, 2}:  # RGB
            self.rgb_renderer.GetRenderWindow().Render()
            if self.iterations == 30:
                self.rgb_renderer.ResetCamera()
        self.iterations -= 1
        self.threadLock.release()


def run(mode=0):
    update_on = threading.Event()
    update_on.set()
    threadLock = threading.Lock()

    pointCloud = VtkPointCloud(mode)
    pointCloud.update(threadLock, update_on)

    visual = Visualization(threadLock, pointCloud, 30)
    visual.renderWindowInteractor.Start()


pipeline = rs.pipeline()
pipeline.start()
run()
