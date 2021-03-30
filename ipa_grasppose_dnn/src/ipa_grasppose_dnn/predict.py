# This file is part of ipa_grasppose_dnn.
# Copyright (C) 2021  Marc Riedlinger

# ipa_grasppose_dnn is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

#!/usr/bin/env python

import rospy
import numpy as np
import tensorflow as tf
import keras.backend as K
from keras.backend.tensorflow_backend import set_session
from keras.models import load_model
from ipa_grasppose_srvs.srv import *
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError


class PredictGraspQuality:

    def __init__(self, predict_params):
        # Check for processor service
        processor_service = "/ipa_graspdata_processor/ipa_graspdata_processor/process_graspdata_srv"
        print("PredictGraspQuality.__init__ - Waiting for processor service...")
        rospy.wait_for_service(processor_service)
        print("PredictGraspQuality.__init__ - Processor service established.")

        # Allow dynamically growing GPU memory to avoid crashes
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        sess = tf.Session(config=config)
        set_session(sess)  # set this TensorFlow session as the default session for Keras

        # Load NN model and set up cv bridge
        self.model = load_model(predict_params["model_filepath"],
                                custom_objects={"focal_loss_fixed": binary_focal_loss()})
        self.default_graph = tf.get_default_graph()
        self.bridge = CvBridge()
        self.image_data_shape = tuple(predict_params["depth_image_dim"])
        self.grasp_data_shape = tuple(predict_params["grasp_data_dim"])

        # Set up prediction service
        self.processor_client = rospy.ServiceProxy(processor_service, ProcessGraspData)
        self.service = rospy.Service("predict_graspquality_srv", PredictGraspQualities, self.predict_grasp_quality)

    def predict_grasp_quality(self, request):
        # Process raw data for grasps
        process_request = ProcessGraspDataRequest(request.depth_image, request.grasp_poses_camera_frame, False,
                                                  Point(0, 0, 0), Vector3(0.2, 0.2, 0.2))
        try:

            response = self.processor_client(process_request)

            predictions = np.zeros(len(response.depth_patches), dtype=float)
            for i in range(len(predictions)):
                try:
                    depth_patch = np.empty((1,) + self.image_data_shape, dtype=float)  # add batch-dimension -> (1,)
                    depth_patch[0, :, :, 0] = self.bridge.imgmsg_to_cv2(response.depth_patches[i], "32FC1")
                    grasp_pose = response.normalized_grasp_poses[i]
                    grasp_pose_np = np.asarray([#grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z,
                                                grasp_pose.orientation.w, grasp_pose.orientation.x,
                                                grasp_pose.orientation.y, grasp_pose.orientation.z])
                    grasp_pose_resized = np.empty((1,) + self.grasp_data_shape)

                    for l in range(self.grasp_data_shape[2]):
                        grasp_pose_resized[0, :, :, l] = np.full(self.grasp_data_shape[0:2], grasp_pose_np[l])

                    nn_input = [depth_patch, grasp_pose_resized]

                    # Fixes error: https://stackoverflow.com/questions/47115946/tensor-is-not-an-element-of-this-graph
                    with self.default_graph.as_default():
                        predictions[i] = self.model.predict(nn_input, batch_size=1)[0, 0]

                except CvBridgeError as e:
                    print(e)

            return PredictGraspQualitiesResponse(predictions)

        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)


# Focal loss to counter effect of data duplicates
# alpha weighs positive class 1 -> 1-alpha is weight for 0
# Taken from: https://github.com/aldi-dimara/keras-focal-loss
# Adjusted to take the mean which is the same keras does with its binary_crossentropy in losses.py
def binary_focal_loss(gamma=2., alpha=.5):
    """
    Binary form of focal loss.
      FL(p_t) = -alpha * (1 - p_t)**gamma * log(p_t)
      where p = sigmoid(x), p_t = p or 1 - p depending on if the label is 1 or 0, respectively.
    References:
        https://arxiv.org/pdf/1708.02002.pdf
    Usage:
     model.compile(loss=[binary_focal_loss(alpha=.5, gamma=2)], metrics=["accuracy"], optimizer=adam)
    """
    def binary_focal_loss_fixed(y_true, y_pred):
        """
        :param y_true: A tensor of the same shape as `y_pred`
        :param y_pred:  A tensor resulting from a sigmoid
        :return: Output tensor.
        """

        epsilon = K.epsilon()
        y_pred = K.clip(y_pred, epsilon, 1. - epsilon) # clip to prevent NaN's and Inf's
        p_t = tf.where(K.equal(y_true, 1), y_pred, 1 - y_pred)
        alpha_factor = K.ones_like(y_true) * alpha
        alpha_t = tf.where(K.equal(y_true, 1), alpha_factor, 1 - alpha_factor)
        return K.mean(K.pow(1. - p_t, gamma) * alpha_t * K.binary_crossentropy(y_true, y_pred), axis=-1)

    return binary_focal_loss_fixed
