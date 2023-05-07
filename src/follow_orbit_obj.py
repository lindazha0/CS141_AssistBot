import pybullet as p
import pybullet_data
import time
import math
import numpy as np

DISTANCE_THRESHOLD = 0.5
REACH_THRESHOLD = 32 # terminate after 32 seconds of reaching target
ORBIT_RADIUS_X = 1.5
ORBIT_RADIUS_Y = 2

# connect to pybullet
client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
print("data path: ", pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

# load plane, robot, and target
plane = p.loadURDF("plane.urdf")
robot = p.loadURDF(
    "../data/turtlebot.urdf", [0, 0, 0], flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL,
    physicsClientId=client
)  # or will collide with ground
# target = p.createVisualhape(p.GEOM_SPHERE, radius=0.1, rgbaColor=[1, 0, 0, 1])
target_obj = p.loadURDF("duck_vhacd.urdf", [0, 0, 0.1])
# target_obj = p.createMultiBody(
#     baseMass=0, baseVisualShapeIndex=target, basePosition=[0, 0, 0.1]
# )

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

# define moving function for object
def move_object(t, obj):
    """Move object along an oval robit over time."""
    target_pos = [ORBIT_RADIUS_X*math.sin(t), ORBIT_RADIUS_Y*math.cos(t), 1]
    p.resetBasePositionAndOrientation(obj, target_pos, [0, 0, 0, 1])
    return target_pos

# define control function for robot
def proportional_control(robot_pos, robot_orn, target_pos, gain):
    """Calculate wheel velocities for proportional control."""
    global hit_time, total_time
    total_time += 1
    # if close enough to target, stop
    if (
        math.sqrt(sum((robot_pos[i] - target_pos[i]) ** 2 for i in range(2)))
        <= DISTANCE_THRESHOLD
    ):
        hit_time += 1
        return 0, 0

    # calculate linear and angular velocity required to reach target
    delta = [target_pos[i] - robot_pos[i] for i in range(2)]  # delta x, delta y
    forward_vel = gain * math.sqrt(sum(e**2 for e in delta))

    # calculate angular velocity required to reach target
    robot_yaw = p.getEulerFromQuaternion(robot_orn)[2]
    angular_err = math.atan2(delta[1], delta[0]) - robot_yaw

    # normalize to -pi to pi
    angular_err = math.atan2(math.sin(angular_err), math.cos(angular_err))
    angular_vel = gain * angular_err

    # convert to wheel velocities
    left_force = forward_vel - angular_vel
    right_force = forward_vel + angular_vel
    return left_force, right_force

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
time_step = 1 / 120
gain = 50
start_time = time.time()

while True:
    # Get the RGB camera's position and orientation
    rgb_camera_pos, rgb_camera_quat = p.getLinkState(robot, rgb_camera_link_index)[:2]

    # Get the depth camera's position and orientation
    depth_camera_pos, depth_camera_quat = p.getLinkState(robot, depth_camera_link_index)[:2]

    # Get the camera view matrix for both cameras
    view_matrix_rgb = p.computeViewMatrix(rgb_camera_pos, rgb_camera_pos + np.array([1, 0, 0]), [0, 0, 1])
    view_matrix_depth = p.computeViewMatrix(depth_camera_pos, depth_camera_pos + np.array([1, 0, 0]), [0, 0, 1])

    # Get the camera projection matrix
    projection_matrix = p.computeProjectionMatrixFOV(fov, width / height, near, far)

    # Get the RGB camera image
    rgb_img = p.getCameraImage(width, height, view_matrix_rgb, projection_matrix)

    # Get the depth camera image
    depth_img = p.getCameraImage(width, height, view_matrix_depth, projection_matrix, flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)

    # Access the RGB and depth images
    rgb_data = np.reshape(rgb_img[2], (height, width, 4))[:, :, :3]
    depth_data = np.reshape(depth_img[3], (height, width))

    # Display the RGB and depth images
    # p.addUserDebugImage(rgb_data, width, height, rgb_camera_pos, rgb_camera_quat)
    # p.addUserDebugImage(depth_data, width, height, depth_camera_pos, depth_camera_quat)

    # update target
    elapsed_time = (time.time() - start_time) / 40
    target_pos = move_object(elapsed_time, target_obj)

    # get robot state and apply control
    robot_pos, robot_orn = p.getBasePositionAndOrientation(robot)
    left_vel, right_vel = proportional_control(robot_pos, robot_orn, target_pos, gain)
    p.setJointMotorControl2(robot, 0, p.VELOCITY_CONTROL, targetVelocity=left_vel, force=1000)
    p.setJointMotorControl2(robot, 1, p.VELOCITY_CONTROL, targetVelocity=right_vel, force=1000)

    # step simulation
    if hit_time >= REACH_THRESHOLD:
        print(f"After {total_time-REACH_THRESHOLD} timesteps, reached target for {REACH_THRESHOLD} timesteps.")
        break
    p.stepSimulation()
    time.sleep(time_step)

# close simulation
p.disconnect()
