import math
import pybullet as p


# define control functions for robot
def base_control(robot_pos, robot_orn, target_pos, gain):
    """Calculate wheel velocities for proportional control."""
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

    # to avoid fall
    # robot_vel, robot_angular_vel = p.getBaseVelocity(robot)

    return left_force, right_force

def predict_object_position(target_pos, filter):
    """Predict the position of the target object."""
    # get object state
    filter.predict()
    filter.update(target_pos[:2])
    estimated_pos = filter.x[:2]

    # predict object position
    pos, vel, acc, dt = filter.x[:2], filter.x[2:4], filter.x[4:], filter.dt*filter.AHEAD_TIME_STEPS
    object_x = estimated_pos[0]+vel[0]*dt+0.5*acc[0]*dt**2
    object_y = estimated_pos[1]+vel[1]*dt+0.5*acc[1]*dt**2
    object_z = target_pos[2]

    return object_x, object_y, object_z

def predict_control(robot_pos, robot_orn, target_pos, gain, filter):
    """Implement predictive modtion control with Kalman Filter."""
    # predict object position
    predict_target_pos = predict_object_position(target_pos, filter)
    return base_control(robot_pos, robot_orn, predict_target_pos, gain)