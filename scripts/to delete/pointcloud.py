import vtk
import random
import numpy as np
import pyrealsense2 as rs
class VtkPointCloud:

    def __init__(self, zMin=-10.0, zMax=10.0, maxNumPoints=1e10):
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

    def addPoint(self, point):
        if self.vtkPoints.GetNumberOfPoints() < self.maxNumPoints:
            pointId = self.vtkPoints.InsertNextPoint(point[:])
            self.vtkDepth.InsertNextValue(point[2])
            self.vtkCells.InsertNextCell(1)
            self.vtkCells.InsertCellPoint(pointId)
        else:
            r = random.randint(0, self.maxNumPoints)
            self.vtkPoints.SetPoint(r, point[:])
        self.vtkCells.Modified()
        self.vtkPoints.Modified()
        self.vtkDepth.Modified()

    def clearPoints(self):
        self.vtkPoints = vtk.vtkPoints()
        self.vtkCells = vtk.vtkCellArray()
        self.vtkDepth = vtk.vtkDoubleArray()
        self.vtkDepth.SetName('DepthArray')
        self.vtkPolyData.SetPoints(self.vtkPoints)
        self.vtkPolyData.SetVerts(self.vtkCells)
        self.vtkPolyData.GetPointData().SetScalars(self.vtkDepth)
        self.vtkPolyData.GetPointData().SetActiveScalars('DepthArray')
class AddPointCloudTimerCallback():
    def __init__(self, renderer, pointcloud):
        # self.iterations = iterations
        self.renderer = renderer
        self.pointcloud= pointcloud

    def execute(self, iren, event):
        # if self.iterations == 0:
        #     iren.DestroyTimer(self.timerId)
        
        
        self.pointcloud.clearPoints()
        iren.GetRenderWindow().Render()
        # if self.iterations == 30:
        #     self.renderer.ResetCamera()

#         self.iterations -= 1
# #

pipeline = rs.pipeline()
pipeline.start()
renderer = vtk.vtkRenderer()
renderer.SetBackground(.2, .3, .4)
renderer.ResetCamera()

# Render Window
renderWindow = vtk.vtkRenderWindow()
renderWindow.AddRenderer(renderer)

pointCloud = VtkPointCloud()
renderer.AddActor(pointCloud.vtkActor)
        
# Interactor
renderWindowInteractor = vtk.vtkRenderWindowInteractor()
renderWindowInteractor.SetRenderWindow(renderWindow)
renderWindowInteractor.Initialize()
#addPointCloudTimerCallback = AddPointCloudTimerCallback(renderer)
# renderWindowInteractor.AddObserver('TimerEvent', addPointCloudTimerCallback.execute)
#timerId = renderWindowInteractor.CreateRepeatingTimer(10)
#addPointCloudTimerCallback.timerId = timerId
renderWindow.Render()

renderWindowInteractor.Start()

while True:
    # Create a pipeline object. This object configures the streaming camera and owns it's handle
    frames = pipeline.wait_for_frames()
    depth = frames.get_depth_frame()
   

    if not depth: continue

    # Print a simple text-based representation of the image, by breaking it into 10x20 pixel regions and approximating the coverage of pixels within one meter
    
    for y in range(480):
        for x in range(640):
            dist = depth.get_distance(x, y)
           # print("point:",[x,y,dist])
            pointCloud.addPoint([x,y,dist])
    addPointCloudTimerCallback = AddPointCloudTimerCallback(renderer,pointCloud)

    renderWindowInteractor.AddObserver('TimerEvent', addPointCloudTimerCallback.execute)
    timerId = renderWindowInteractor.CreateRepeatingTimer(10)
    #addPointCloudTimerCallback.timerId = timerId


    

