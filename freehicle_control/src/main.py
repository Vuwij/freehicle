#!/usr/bin/env python3

from os.path import expanduser

import cv2
import numpy as np
import tf
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo

import pybullet as pb
import pybullet_data
import rospy

from soccer_common.transformation import Transformation

if __name__ == "__main__":
    rospy.init_node("freehicle_control")
    pub_camera = rospy.Publisher("camera/image_raw", Image, queue_size=1)
    pub_camera_info = rospy.Publisher("camera/camera_info", CameraInfo, queue_size=1, latch=True)

    pub_odometry = rospy.Publisher("odom", Odometry, queue_size=1)
    pub_odometry_gt = rospy.Publisher("odom_ground_truth", Odometry, queue_size=1)

    rospy.set_param("/use_sim_time", False)

    client_id = pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    pb.setGravity(0, 0, -9.81)
    # pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
    home = expanduser("~")

    plane = pb.loadURDF(
        home
        + f"/catkin_ws/src/freehicle/freehicle_description/urdf/parking_lot/parking_lot.urdf"
        , basePosition=[0, 0, 0], baseOrientation=[0, 0, 0, 1])
    pb.changeDynamics(
        plane,
        linkIndex=-1,
        lateralFriction=1.0,
        spinningFriction=0,
        rollingFriction=0.0
    )

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

    projection_matrix = pb.computeProjectionMatrixFOV(fov, aspect, near, far)

    br = tf.TransformBroadcaster()

    t = 0
    r = rospy.Rate(1 / 0.004)
    cvbridge = CvBridge()

    pos_prev = Transformation()
    pose_start = Transformation()
    while(True):
        # Get depth values using the OpenGL renderer
        camera_pos = pb.getLinkState(pybullet_id, 4)
        camera_transform = Transformation(camera_pos[0], camera_pos[1])
        target_position = Transformation(position=[1, 0, -0.5])
        target_camera_position = camera_transform @ target_position

        base_pos = pb.getLinkState(pybullet_id, 0)

        pos = Transformation(position=base_pos[0], quaternion=base_pos[1])
        pos_diff = np.linalg.inv(pos_prev) @ pos
        pos_prev = pos
        rand = np.random.default_rng().multivariate_normal([0, 0, 0], np.diag([0.00001, 0, 0.0001]))

        pose_start = pose_start @ pos_diff @ Transformation(pos_theta=rand)

        o = Odometry()
        o.header.frame_id = "map"
        o.header.stamp = rospy.Time.now()
        o.pose.pose = pose_start.pose
        pub_odometry.publish(o)


        o = Odometry()
        o.header.frame_id = "map"
        o.header.stamp = rospy.Time.now()
        o.pose.pose = pos.pose
        pub_odometry_gt.publish(o)

        br.sendTransform(pos.position,
                         pos.quaternion,
                         rospy.Time.now(),
                         "body",
                         "map")


        view_matrix = pb.computeViewMatrix(cameraEyePosition=camera_pos[0], cameraTargetPosition=target_camera_position.position, cameraUpVector=[0, 0, 1])
        width, length, rgbPixels, depthPixels, segmentationMaskBuffer = pb.getCameraImage(width, height, view_matrix, projection_matrix, renderer=pb.ER_BULLET_HARDWARE_OPENGL)
        rgbPixels = cv2.cvtColor(rgbPixels, cv2.COLOR_RGBA2RGB)

        img_msg = cvbridge.cv2_to_imgmsg(rgbPixels, encoding="bgr8")
        pub_camera.publish(img_msg)

        if t < 1:
            control = [0, 10]
        elif t < 2.5:
            control = [-1, 10]
        elif t < 3.0:
            control = [1, 0]
        elif t < 4.5:
            control = [1, -3]
        elif t < 6.0:
            control = [-0.3, 7]
        elif t < 8.0:
            control = [0, 4]
        elif t < 11.0:
            control = [0, 0]

        pb.setJointMotorControlArray(
            bodyIndex=pybullet_id,
            controlMode=pb.VELOCITY_CONTROL,
            jointIndices=[1, 3, 4, 5],
            targetVelocities=[
                control[1],
                control[1],
                control[1],
                control[1],
            ],
            velocityGains=[0.4] * 4,
        )
        pb.setJointMotorControlArray(
            bodyIndex=pybullet_id,
            controlMode=pb.POSITION_CONTROL,
            jointIndices=[0, 2],
            targetPositions=[
                control[0],
                control[0]
            ]
        )
        pb.stepSimulation()
        r.sleep()
        t = t + 0.004