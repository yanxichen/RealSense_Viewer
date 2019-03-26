import vtk
import numpy as np
from numpy import random
import pyrealsense2 as rs


class VtkPointCloud:

    def __init__(self, zMin=0.0, zMax=255.0, maxNumPoints=1e10):
        self.maxNumPoints = maxNumPoints
        self.vtkPolyData = vtk.vtkPolyData()
        self.clearPoints()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(self.vtkPolyData)
        mapper.SetColorModeToDefault()
        mapper.SetScalarRange(zMin, zMax)
        mapper.SetScalarVisibility(1)
        self.vtkActor = vtk.vtkActor()
        self.vtkActor.SetMapper(mapper)
        self.numPoints = 0

    def addPoint(self, point):
        if self.vtkPoints.GetNumberOfPoints() < self.maxNumPoints:
            pointId = self.vtkPoints.InsertNextPoint([640 - point[0], 480 - point[1], 0 - point[2]])
            self.vtkDepth.InsertNextValue(point[2])
            self.vtkCells.InsertNextCell(1)
            self.vtkCells.InsertCellPoint(pointId)
        else:
            r = random.randint(0, self.maxNumPoints)
            self.vtkPoints.SetPoint(r, point[:])
        self.vtkCells.Modified()
        self.vtkPoints.Modified()
        self.vtkDepth.Modified()
        self.numPoints += 1

    def clearPoints(self):
        self.vtkPoints = vtk.vtkPoints()
        self.vtkCells = vtk.vtkCellArray()
        self.vtkDepth = vtk.vtkDoubleArray()
        self.vtkDepth.SetName('DepthArray')
        self.vtkPolyData.SetPoints(self.vtkPoints)
        self.vtkPolyData.SetVerts(self.vtkCells)
        self.vtkPolyData.GetPointData().SetScalars(self.vtkDepth)
        self.vtkPolyData.GetPointData().SetActiveScalars('DepthArray')


pipeline = rs.pipeline()
pipeline.start()


def updatedata(pointCloud):
    pointCloud.clearPoints()
    frames = pipeline.wait_for_frames()
    depth = frames.get_depth_frame()
    if not depth:
        pass

    for y in range(480):
        for x in range(640):
            dist = depth.get_distance(x, y) * 50
            if x % 3 == 0 and y % 3 == 0:
                pointCloud.addPoint([x, y, dist])


class AddPointCloudTimerCallback:
    def __init__(self, renderer, iterations):
        self.iterations = iterations
        self.renderer = renderer

    def execute(self, iren, event):
        pointCloud = VtkPointCloud()
        self.renderer.AddActor(pointCloud.vtkActor)
        pointCloud.clearPoints()
        updatedata(pointCloud)
        iren.GetRenderWindow().Render()
        if self.iterations == 30:
            self.renderer.ResetCamera()
        self.iterations -= 1
        self.renderer.RemoveActor(pointCloud.vtkActor)


colors = vtk.vtkUnsignedCharArray()
colors.SetNumberOfComponents(3)
colors.SetName("colors")

renderer = vtk.vtkRenderer()
renderer.SetBackground(.2, .3, .4)
renderer.ResetCamera()

renderWindow = vtk.vtkRenderWindow()
renderWindow.SetSize(1000, 800)
renderWindow.AddRenderer(renderer)

# Interactor
renderWindowInteractor = vtk.vtkRenderWindowInteractor()
renderWindowInteractor.SetRenderWindow(renderWindow)
renderWindowInteractor.Initialize()

# Initialize a timer for the animation
addPointCloudTimerCallback = AddPointCloudTimerCallback(renderer, 30)
renderWindowInteractor.AddObserver('TimerEvent', addPointCloudTimerCallback.execute)
timerId = renderWindowInteractor.CreateRepeatingTimer(1)

# Begin Interaction
renderWindow.Render()
renderWindowInteractor.Start()
