#!/usr/bin/env python
import numpy as np
import cv2
import cv2.aruco as aruco
import math
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, Point
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler, euler_from_quaternion

"""
This demo calculates multiple things for different scenarios.
Here are the defined reference frames:
TAG:
                A y
                |
                |
                |tag center
                O---------> x
CAMERA:
                X--------> x
                | frame center
                |
                |
                V y
F1: Flipped (180 deg) tag frame around x axis
F2: Flipped (180 deg) camera frame around x axis
The attitude of a generic frame 2 respect to a frame 1 can obtained by calculating euler(R_21.T)
We are going to obtain the following quantities:
    > from aruco library we obtain tvec and Rct, position of the tag in camera frame and attitude of the tag
    > position of the Camera in Tag axis: -R_ct.T*tvec
    > Transformation of the camera, respect to f1 (the tag flipped frame): R_cf1 = R_ct*R_tf1 = R_cf*R_f
    > Transformation of the tag, respect to f2 (the camera flipped frame): R_tf2 = Rtc*R_cf2 = R_tc*R_f
    > R_tf1 = R_cf2 an symmetric = R_f
"""


class ArucoDetector(object):

    def __init__(self):

        #--- Define Tag
        self.id_to_find  = 20
        self.marker_size  = 0.5
        #rospy.init_node("ArucoInvertedPublisher")
        #self.pub = rospy.Publisher("ArucoPose", PoseStamped, queue_size=1)

        #self.hdr = Header()
        #self.hdr.frame_id = "camera"
        #self.hdr.stamp = rospy.Time.now()

    def is_rotation_matrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6


    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    def rotation_matrix_to_euler_angles(self, R):
        assert (self.is_rotation_matrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])


    def extract_cameracalib(self):
        ''' Gets the the camera and distortion matrix from the calibrate_camera method. By reading the yaml file.'''
        cv_file = cv2.FileStorage("calibration.yaml", cv2.FILE_STORAGE_READ)
        print cv_file
        #print 'Type from read file:', type(cv_file)
        # Note we also have to specify the type to retrieve other wise we only get a
        # FileNode object back instead of a matrix
        camera_matrix = cv_file.getNode("camera_matrix").mat()
        dist_matrix = cv_file.getNode("dist_coeff").mat()
        cv_file.release()
        return camera_matrix, dist_matrix

    def track_aruco(self):

        # Since we want our Marker reference frame to be aligned with the camera frame, we rotate it by 180 degrees wrt X-Axis
        R_flip_180 = np.zeros((3,3), dtype=np.float32)
        R_flip_180[0,0] = 1.0
        R_flip_180[1,1] = -1.0
        R_flip_180[2,2] = -1.0

        aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        parameters  = aruco.DetectorParameters_create()

        camera_matrix, camera_distortion = self.extract_cameracalib()
        cap = cv2.VideoCapture(1)
        # Checar e ver se isso faz diferenca dps!
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        #-- Font for the text in the image
        font = cv2.FONT_HERSHEY_PLAIN

        while True:
            ret, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            #-- Find all the aruco markers in the image
            corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,
                                      cameraMatrix=camera_matrix, distCoeff=camera_distortion)

            #if ids != None and ids[0] == id_to_find:
            if np.all(ids != None):

                #ret = aruco.estimatePoseSingleMarkers(corners, self.marker_size, camera_matrix, camera_distortion)

                # Unpacking the values for the rot and trans vectors
                #rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, camera_matrix, camera_distortion)
                print ("Tvec Antes da inversao: \n"), tvec
                print ("Rvec Antes da inversao: \n"), rvec
                # Draw the detected marker and put the reference frame OF THE MARKER on it
                aruco.drawDetectedMarkers(frame, corners)
                aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)

                # By giving the euler angles, we obtain the rotation matrix marker->camera
                R = np.matrix(cv2.Rodrigues(rvec)[0])

                # Then we get the inverse of it, which is the same as the transpose
                # Meaning now we have the rotation matrix from: camera->marker
                Rt = R.T

                # Applying the 180 degrees so that the frames are aligned
                R_final = R_flip_180*Rt

                # Converts the rot matrix to euler angles again
                rpy = self.rotation_matrix_to_euler_angles(R_final)

                # In order to obtain the inverse of the transformation camera->marker, we have to inverse the translation as well
                pos_camera = -Rt*np.matrix(tvec).T

                # Converting to meters
                pos_camera = pos_camera*0.1

                #print rpy
                #self.publishRos(rpy, pos_camera)

            cv2.imshow('frame', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                cap.release()
                cv2.destroyAllWindows()
                break

    def publishRos(self, invRvec, invTvec):
        position = Point(invTvec[0], invTvec[1], invTvec[2])
        init_pos = Pose()
        init_pos.position = position

        quat = quaternion_from_euler(invRvec[0], invRvec[1], invRvec[2])
        quat_tupl = Quaternion(quat[0], quat[1], quat[2], quat[3])
        init_pos.orientation = quat_tupl
        ps = PoseStamped(self.hdr, init_pos)
        self.pub.publish(ps)


if __name__ == "__main__":
    ad = ArucoDetector()
    ad.track_aruco()


