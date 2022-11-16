from enum import Enum
import bpy
import os
from math import radians
scene = bpy.data.scenes.new("Scene")
camera_data = bpy.data.camera.new("Camera")

camera = bpy.data.objects.new("Camera", camera_data)
camera.location = (-2.0, 3.0, 3.0)
camera.rotation_euler = ([radians(a) for a in (422.0, 0.0, 149)])
scene.objects.link(camera)
class Bricks(Enum):
    brick0 = 0
    brick1 = 1
    brick2 = 2
    brick3 = 3
    brick4 = 4
    brick5 = 5
    brick6 = 6
    brick7 = 7
    brick8 = 8
    brick9 = 9
    brick10 = 10

brickNames = {
    "X1-Y1-Z2",
    "X1-Y2-Z1",
    "X1-Y2-Z2-CHAMFER",
    "X1-Y2-Z2-TWINFILLET",
    "X1-Y2-Z2",
    "X1-Y3-Z2-FILLET",
    "X1-Y3-Z2",
    "X1-Y4-Z1",
    "X1-Y4-Z2",
    "X2-Y2-Z2-FILLET",
    "X2-Y2-Z2"
}

bpy.data.scenes[0]

## TOCCA SMONTARE DAL TRENO DC, CHE SCHIFO BLENDER
for i in range(brickNames):
    #Import the mesh into the scene
    bpy.ops.import_mesh.stl(filepath=brickNames[i]+".stl")
    


