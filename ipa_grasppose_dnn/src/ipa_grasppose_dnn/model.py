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

from keras.models import Model
from keras.layers import Input, Dense, Conv2D, MaxPool2D, BatchNormalization, Activation, Concatenate, Flatten, Dropout

# Model created during Master's thesis.
# This model requires several changes:
# - model = get_dnn_model(image_input_shape, grasp_data_input_shape) in training.py
# - "depth_image_dim": [64, 64, 1] in ipa_train_grasp_dnn and ipa_process_results
# - "batch_size": 192 in ipa_train_grasp_dnn and ipa_process_results
# - Set num_outputs = 1 in ipa_process_results
# - comment out downscaling to 32x32 in datagen.py (__data_generation())
#   # depth_patch = cv2.resize(depth_patch, (32,32), interpolation=cv2.INTER_AREA)
def get_dnn_model(image_input_shape=(64, 64, 1), grasp_data_input_shape=(8, 8, 4)):
    initializer = "glorot_uniform"
    num_outputs = 1
    x = image_input = Input(shape=image_input_shape, name="image_input")
    x = conv_block(x, 64, init=initializer)
    x = conv_block(x, 64, init=initializer)
    x = MaxPool2D(pool_size=(2, 2), strides=(2, 2), padding="same")(x)
    x = conv_block(x, 128, init=initializer)
    x = conv_block(x, 128, init=initializer)
    x = MaxPool2D(pool_size=(2, 2), strides=(2, 2), padding="same")(x)
    x = conv_block(x, 256, init=initializer)
    x = conv_block(x, 256, init=initializer)
    x = conv_block(x, 256, (1, 1), "valid", init=initializer)
    x = MaxPool2D(pool_size=(2, 2), strides=(2, 2), padding="same")(x)  # last layer of CNN tower

    # Build the grasp data processing branch
    grasp_input = Input(shape=grasp_data_input_shape, name="grasp_input")

    # Merge both branches and process unified branch
    x = Concatenate(axis=3)([x, grasp_input])  # concatenate around feature dimension
    x = conv_block(x, 512, init=initializer)
    x = conv_block(x, 512, init=initializer)
    x = conv_block(x, 512, (1, 1), "valid", init=initializer)
    x = MaxPool2D(pool_size=(2, 2), strides=(2, 2), padding="same")(x)

    x = Flatten()(x)
    x = Dropout(0.5)(x)
    x = Dense(units=1024, activation="relu", kernel_initializer=initializer)(x)
    x = Dense(units=num_outputs, kernel_initializer=initializer)(x)
    x = Activation("sigmoid")(x)

    # Create keras model from tensors
    return num_outputs, Model(inputs=[image_input, grasp_input], outputs=x)


# Model for comparison only.
# This model requires several changes:
# - model = get_improved_dexnet_model(image_input_shape, grasp_data_input_shape) in training.py
# - "depth_image_dim": [32, 32, 1] in ipa_train_grasp_dnn and ipa_process_results
# - "batch_size": 512 in ipa_train_grasp_dnn and ipa_process_results
# - Set num_outputs = 2 in ipa_process_results
# - uncomment downscaling to 32x32 in datagen.py (__data_generation())
#   depth_patch = cv2.resize(depth_patch, (32, 32), interpolation=cv2.INTER_AREA)
def get_improved_dexnet_model(image_input_shape=(32, 32, 1), grasp_data_input_shape=(8, 8, 4)):
    initializer = "glorot_uniform"
    num_outputs = 2
    x = image_input = Input(shape=image_input_shape, name="image_input")
    x = conv_block(x, 64, init=initializer)
    x = conv_block(x, 128, init=initializer)
    x = MaxPool2D(pool_size=(2, 2), strides=(2, 2), padding="same")(x)
    x = conv_block(x, 128, init=initializer)
    x = conv_block(x, 256, init=initializer)
    x = MaxPool2D(pool_size=(2, 2), strides=(2, 2), padding="same")(x)

    # Build the grasp data processing branch
    grasp_input = Input(shape=grasp_data_input_shape, name="grasp_input")

    # Merge both branches and process unified branch
    x = Concatenate(axis=3)([x, grasp_input])  # concatenate around feature dimension
    x = conv_block(x, 256, init=initializer)
    x = conv_block(x, 256, init=initializer)

    x = Flatten()(x)
    x = Dropout(0.5)(x)
    x = Dense(units=512, activation="relu", kernel_initializer=initializer)(x)
    x = Dense(units=num_outputs, kernel_initializer=initializer)(x)
    x = Activation("softmax")(x)

    # Create keras model from tensors
    return num_outputs, Model(inputs=[image_input, grasp_input], outputs=x)


def conv_block(x, num_filters, kernel=(3, 3), padding="same", init="glorot_uniform"):
    x = Conv2D(filters=num_filters, kernel_size=kernel, kernel_initializer=init, strides=(1, 1), padding=padding)(x)
    x = BatchNormalization()(x)
    x = Activation("relu")(x)
    return x
