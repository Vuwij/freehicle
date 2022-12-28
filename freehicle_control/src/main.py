#!/usr/bin/env python3

import os
import time
from os.path import expanduser

from sensor_msgs.msg import Image, CameraInfo

import pybullet as pb
import pybullet_data
import rospy

if __name__ == "__main__":
    rospy.init_node("freehicle_control")
    pub_camera = rospy.Publisher("camera/image_raw", Image, queue_size=1)
    pub_camera_info = rospy.Publisher("camera/camera_info", CameraInfo, queue_size=1, latch=True)

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
        useFixedBase=False,
        flags=pb.URDF_USE_INERTIA_FROM_FILE,
        basePosition=[0, 0, 0.1016],
        baseOrientation=[0, 0, 0, 1],
    )
    motor_names = [pb.getJointInfo(pybullet_id, i)[1].decode("utf-8") for i in range(pb.getNumJoints(pybullet_id))]
    print(motor_names)

    width = 640
    height = 480

    fov = 60
    aspect = width / height
    near = 0.02
    far = 10

    view_matrix = pb.computeViewMatrix([0, 0, 1], [2, 0, 0], [1, 0, 0])
    projection_matrix = pb.computeProjectionMatrixFOV(fov, aspect, near, far)


    while(True):
        # Get depth values using the OpenGL renderer
        width, length, rgbPixels, depthPixels, segmentationMaskBuffer = pb.getCameraImage(width, height, view_matrix, projection_matrix, renderer=pb.ER_BULLET_HARDWARE_OPENGL)

        img_msg = Image()
        img_msg.header.stamp = rospy.Time.now()
        img_msg.header.frame_id = "/camera"
        img_msg.height = height
        img_msg.width = width
        img_msg.encoding = b"bgra8"
        img_msg.step = 4 * width
        img_msg.data = rgbPixels
        pub_camera.publish(img_msg)

        time.sleep(0.01)

        control = [1, 1, 1]
        pb.setJointMotorControlArray(
            bodyIndex=pybullet_id,
            controlMode=pb.VELOCITY_CONTROL,
            jointIndices=[0, 1, 2, 3, 4, 5],
            targetVelocities=[
                control[0],
                control[1],
                control[0],
                control[1],
                control[2],
                control[2],
            ],
            velocityGains=[0.4] * 6,
        )
        pb.stepSimulation()