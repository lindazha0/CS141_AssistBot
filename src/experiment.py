import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import utils as ut
from sensors import connect_cameras
from control import *
from kalmanFilter import create_kalman_filter

DISTANCE_THRESHOLD = 0.5
REACH_THRESHOLD = 32    # terminate after 32 seconds of reaching target
AGENT = "predictive"      # "baseline" or "predictive"
TIME_STEP = 1 / 120
KEYBOARD_CONTROL = False
ORBIT_RADIUS_X = 1.5
ORBIT_RADIUS_Y = 2

# connect to pybullet
client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

# load plane, robot, and target
plane = p.loadURDF("plane.urdf")
robot = ut.load_robot()
target_obj = ut.load_object("sphere")

# set camera
p.resetDebugVisualizerCamera(
    cameraDistance=5.0, cameraYaw=180, cameraPitch=-90, cameraTargetPosition=[0, 0, 0]
)

# obtain robot joints information
num_joints = p.getNumJoints(robot)
for i in range(num_joints):
    joint_info = p.getJointInfo(robot, i)
    print(f"Joint index: {i}, Joint name: {joint_info[1].decode('utf-8')}")

# define criteria for the experiment
hit_time, total_time = 0, 0


# configure camera settings on turtlebot
width = 320
height = 240
fov = 60
near = 0.02
far = 100

# Set the camera link index
rgb_camera_link_index = 28
depth_camera_link_index = 31

# run simulation
gain = 50 # proportional control gain, or speed
start_time = time.time()
camera_T = 4 # camera update rate
object_T = 60 # object update rate
if AGENT == "predictive":
    filter = create_kalman_filter(TIME_STEP)
    filter.x = np.array([0, 0, 0, 0, 0, 0])
    filter.dt = TIME_STEP

while True:
    # update camera
    total_time += 1
    if total_time % camera_T == 0:
        connect_cameras(robot)

    # update target
    elapsed_time = (time.time() - start_time) / object_T
    target_pos = ut.move_object(elapsed_time, target_obj, KEYBOARD_CONTROL)

    # get robot state and apply control
    robot_pos, robot_orn = p.getBasePositionAndOrientation(robot)

    # if close enough to target, stop
    if (
        math.sqrt(sum((robot_pos[i] - target_pos[i]) ** 2 for i in range(2))) <= DISTANCE_THRESHOLD
    ):
        hit_time += 1
    else:
        # move towards target
        if AGENT == "predictive":
            left_vel, right_vel = predict_control(robot_pos, robot_orn, target_pos, gain, filter)
        elif AGENT == "baseline":
            left_vel, right_vel = base_control(robot_pos, robot_orn, target_pos, gain)
        else:
            raise ValueError(f"Invalid agent type: {AGENT}, must be 'baseline' or 'predictive'")
        p.setJointMotorControl2(robot, 0, p.VELOCITY_CONTROL, targetVelocity=left_vel, force=1000)
        p.setJointMotorControl2(robot, 1, p.VELOCITY_CONTROL, targetVelocity=right_vel, force=1000)

    # step simulation
    if hit_time >= REACH_THRESHOLD:
        print(f"After {total_time-REACH_THRESHOLD} timesteps, reached target for {REACH_THRESHOLD} timesteps.")
        break
    p.stepSimulation()
    time.sleep(TIME_STEP)

# close simulation
p.disconnect()
