import pybullet as p
import numpy as np

# configure camera settings on turtlebot
width = 320
height = 240
fov = 60
near = 0.02
far = 100

# Set the camera link index
rgb_camera_link_index = 28
depth_camera_link_index = 31

def connect_cameras(robot):
    # Get position and orientation for the RGB camera and depth camera
    rgb_camera_pos, rgb_camera_quat = p.getLinkState(robot, rgb_camera_link_index)[:2]
    depth_camera_pos, depth_camera_quat = p.getLinkState(robot, depth_camera_link_index)[:2]

    # Get the camera view matrix for both cameras
    view_matrix_rgb = p.computeViewMatrix(rgb_camera_pos, rgb_camera_pos + np.array([1, 0, 0]), [0, 0, 1])
    view_matrix_depth = p.computeViewMatrix(depth_camera_pos, depth_camera_pos + np.array([1, 0, 0]), [0, 0, 1])

    # Get projection matrix for both cameras
    projection_matrix = p.computeProjectionMatrixFOV(fov, width / height, near, far)

    # Get both camera images
    rgb_img = p.getCameraImage(width, height, view_matrix_rgb, projection_matrix)
    depth_img = p.getCameraImage(width, height, view_matrix_depth, projection_matrix, flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)

    # Access the RGB and depth images
    # rgb_data = np.reshape(rgb_img[2], (height, width, 4))[:, :, :3]
    # depth_data = np.reshape(depth_img[3], (height, width))
