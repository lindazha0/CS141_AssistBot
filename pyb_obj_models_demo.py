import os
import time
import pybullet as p
import pybullet_data
from pybullet_object_models import ycb_objects

# Open GUI and set pybullet_data in the path
p.connect(p.GUI)
p.resetDebugVisualizerCamera(3, 90, -30, [0.0, -0.0, -0.0])
p.setTimeStep(1 / 240.)

# Load plane contained in pybullet_data
planeId = p.loadURDF(os.path.join(
    pybullet_data.getDataPath(), "plane.urdf"))

flags = p.URDF_USE_INERTIA_FROM_FILE
obj_id = p.loadURDF(os.path.join(ycb_objects.getDataPath(
), 'YcbMustardBottle', "model.urdf"), [1., 0.0, 0.8], flags=flags)

p.setGravity(0, 0, -9.8)

while 1:
    p.stepSimulation()
    time.sleep(1./240)
