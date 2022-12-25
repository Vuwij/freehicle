#!/usr/bin/env python3

import os
import time
from os.path import expanduser

from matplotlib import pyplot as plt

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot0"

import pybullet as pb
import pybullet_data
import rospy

if __name__ == "__main__":
    rospy.init_node("freehicle_control")

    client_id = pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    pb.setGravity(0, 0, -9.81)
    # pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
    plane = pb.loadURDF("plane.urdf", basePosition=[0, 0, 0], baseOrientation=[0, 0, 0, 1])
    pb.changeDynamics(
        plane,
        linkIndex=-1,
        lateralFriction=0.9,
        spinningFriction=0.9,
        rollingFriction=0.0
    )

    home = expanduser("~")
    pybullet_id = pb.loadURDF(
        home
        + f"/catkin_ws/src/freehicle/freehicle_description/urdf/freehicle/freehicle.urdf",
        useFixedBase=True,
        flags=pb.URDF_USE_INERTIA_FROM_FILE,
        basePosition=[0, 0, 0.1016],
        baseOrientation=[0, 0, 0, 1],
    )

    width = 640
    height = 480

    fov = 60
    aspect = width / height
    near = 0.02
    far = 10

    view_matrix = pb.computeViewMatrix([0, 0, 1], [2, 0, 0], [1, 0, 0])
    projection_matrix = pb.computeProjectionMatrixFOV(fov, aspect, near, far)

    # Get depth values using the OpenGL renderer
    images = pb.getCameraImage(width, height, view_matrix, projection_matrix, renderer=pb.ER_BULLET_HARDWARE_OPENGL)
    depth_buffer_opengl = pb.reshape(images[3], [width, height])
    depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)

    # Get depth values using Tiny renderer
    images = pb.getCameraImage(width, height, view_matrix, projection_matrix, renderer=pb.ER_TINY_RENDERER)
    depth_buffer_tiny = pb.reshape(images[3], [width, height])
    depth_tiny = far * near / (far - (far - near) * depth_buffer_tiny)

    pb.disconnect()

    # Plot both images - should show depth values of 0.45 over the cube and 0.5 over the plane
    plt.subplot(1, 2, 1)
    plt.imshow(depth_opengl, cmap='gray', vmin=0, vmax=1)
    plt.title('OpenGL Renderer')

    plt.subplot(1, 2, 2)
    plt.imshow(depth_tiny, cmap='gray', vmin=0, vmax=1)
    plt.title('Tiny Renderer')

    plt.show()
    
    while(True):
        pb.stepSimulation()
        time.sleep(0.01)
