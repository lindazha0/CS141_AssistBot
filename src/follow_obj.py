import pybullet as p
import pybullet_data
import time
import math

DISTANCE_THRESHOLD = 1.0

# connect to pybullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# print("data path: ", pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

# load plane, robot, and target
plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("../data/turtlebot/turtlebot.urdf", [0, 0, 0])  # or will collide with ground
target = p.createVisualShape(p.GEOM_SPHERE, radius=0.1, rgbaColor=[1, 0, 0, 1])
target_obj = p.createMultiBody(
    baseMass=0, baseVisualShapeIndex=target, basePosition=[0, 0, 0.1])

# obtain robot joints information
num_joints = p.getNumJoints(robot)
for i in range(num_joints):
    joint_info = p.getJointInfo(robot, i)
    print(f"Joint index: {i}, Joint name: {joint_info[1].decode('utf-8')}")

# define control function
def proportional_control(robot_pos, robot_orn, target_pos, gain):
    """Calculate wheel velocities for proportional control."""
    # if close enough to target, stop
    if math.sqrt(sum((robot_pos[i] - target_pos[i])**2 for i in range(2))) <= DISTANCE_THRESHOLD:
        return 0, 0

    # calculate linear and angular velocity required to reach target
    delta = [target_pos[i] - robot_pos[i] for i in range(2)] # delta x, delta y
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


# run simulation
time_step = 1/120
gain = 20
start_time = time.time()

# Set lateral friction for R2D2's wheels
# p.changeDynamics(robot, 2, lateralFriction=1)
# p.changeDynamics(robot, 3, lateralFriction=1)
# p.changeDynamics(robot, 6, lateralFriction=1)
# p.changeDynamics(robot, 7, lateralFriction=1)

target_pos = [2, 2, 1]
p.resetBasePositionAndOrientation(target_obj, target_pos, [0, 0, 0, 1])

while True:
    # update target
    elapsed_time = (time.time() - start_time) / 8
    target_pos = [math.sin(elapsed_time), math.cos(elapsed_time), 1]
    p.resetBasePositionAndOrientation(target_obj, target_pos, [0, 0, 0, 1])

    # get robot state
    robot_pos, robot_orn = p.getBasePositionAndOrientation(robot)

    # apply control
    # apply control
    left_vel, right_vel = proportional_control(robot_pos, robot_orn, target_pos, gain)
    p.setJointMotorControl2(robot, 0, p.VELOCITY_CONTROL, targetVelocity=left_vel)
    p.setJointMotorControl2(robot, 1, p.VELOCITY_CONTROL, targetVelocity=right_vel)

    # step simulation
    p.stepSimulation()
    time.sleep(time_step)
