import vtk
import numpy as np
from numpy import random
import pyrealsense2 as rs
import threading
import time


class VtkPointCloud:
    def __init__(self, color=False, zMin=0.0, zMax=255.0, maxNumPoints=1e10):
        self.maxNumPoints = maxNumPoints
        self.vtkPolyData = vtk.vtkPolyData()
        self.color = color
        mapper = vtk.vtkPolyDataMapper()
        if self.color is False:
            mapper.SetColorModeToDefault()
            mapper.SetScalarRange(zMin, zMax)
        mapper.SetScalarVisibility(1)
        mapper.SetInputData(self.vtkPolyData)
        self.clearPoints()
        self.vtkActor = vtk.vtkActor()
        self.vtkActor.SetMapper(mapper)
        self.numPoints = 0  # For debugging

    def addPoint(self, point):
        pointId = self.vtkPoints.InsertNextPoint([640 - point[0], 480 - point[1], 0 - point[2]])
        self.vtkDepth.InsertNextValue(point[2])
        self.vtkCells.InsertNextCell(1)
        self.vtkCells.InsertCellPoint(pointId)
        self.vtkPolyData.Modified()
        self.vtkCells.Modified()
        self.vtkPoints.Modified()
        self.vtkDepth.Modified()
        # self.numPoints += 1

    def clearPoints(self):
        self.vtkPoints = vtk.vtkPoints()
        self.vtkCells = vtk.vtkCellArray()
        self.vtkDepth = vtk.vtkDoubleArray()
        self.vtkDepth.SetName('DepthArray')
        self.vtkPolyData.SetPoints(self.vtkPoints)
        self.vtkPolyData.SetVerts(self.vtkCells)
        if self.color is False:
            self.vtkPolyData.GetPointData().SetScalars(self.vtkDepth)
            self.vtkPolyData.GetPointData().SetActiveScalars('DepthArray')
        else:
            self.Colors = vtk.vtkUnsignedCharArray()
            self.Colors.SetNumberOfComponents(3)
            self.Colors.SetName("Colors")
            self.vtkPolyData.GetPointData().SetScalars(self.Colors)
            self.vtkPolyData.GetPointData().SetActiveScalars('Colors')
        # self.numPoints = 0

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
                    if self.color:
                        rgb = color_data[y, x]
                        self.Colors.InsertNextTuple3(rgb[0], rgb[1], rgb[2])
            threadLock.release()
            # print(self.numPoints)

    def update(self, threadLock, update_on):
        thread = threading.Thread(target=self.update_pointcloud, args=(threadLock, update_on))
        thread.start()


class Visualization:
    def __init__(self, threadLock, pointCloud, iterations):
        self.threadLock = threadLock
        self.iterations = iterations
        self.pointCloud = pointCloud

        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(.2, .3, .4)
        self.renderer.ResetCamera()
        self.renderer.AddActor(pointCloud.vtkActor)

        self.renderWindow = vtk.vtkRenderWindow()
        self.renderWindow.SetSize(1000, 800)
        self.renderWindow.AddRenderer(self.renderer)

        # Interactor
        self.renderWindowInteractor = vtk.vtkRenderWindowInteractor()
        self.renderWindowInteractor.SetRenderWindow(self.renderWindow)
        self.renderWindowInteractor.Initialize()

        self.renderWindowInteractor.AddObserver('TimerEvent', self.update_visualization)
        timerId = self.renderWindowInteractor.CreateRepeatingTimer(30)

    def update_visualization(self, obj=None, event=None):
        time.sleep(0.01)
        self.threadLock.acquire()
        self.renderer.GetRenderWindow().Render()
        if self.iterations == 30:
            self.renderer.ResetCamera()
            self.iterations -= 1
        self.threadLock.release()


pipeline = rs.pipeline()
pipeline.start()

update_on = threading.Event()
update_on.set()
threadLock = threading.Lock()

pointCloud = VtkPointCloud(color=True)  # Default to heatmap when color=False
pointCloud.update(threadLock, update_on)

visual = Visualization(threadLock, pointCloud, 30)
visual.renderWindowInteractor.Start()
