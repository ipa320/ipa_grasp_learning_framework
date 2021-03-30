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

from model import get_dnn_model
from model import get_improved_dexnet_model  # For evaluation purpose.
from datagen import GraspDataGenerator
import tensorflow as tf
import keras.backend as K
from keras.backend.tensorflow_backend import set_session
from keras import optimizers
from keras.callbacks import Callback
from keras.callbacks import CSVLogger
from keras.models import load_model
from time import time
import os
# os.environ["CUDA_DEVICE_ORDER"] = "0"  # switch to CPU

def train_model(databases, generator_params, train_params):

    file_path = train_params["file_path"]
    learning_rate = train_params["start_lr"]

    new_session = True
    if os.path.isfile(file_path):
        new_session = False

    epoch_offset = 0
    if new_session:
        # Create storage folder for model saving
        if not os.path.isdir(file_path):
            os.mkdir(file_path)

        sessions = [os.path.join(file_path, sess) for sess in os.listdir(file_path)
                    if os.path.isdir(os.path.join(file_path, sess))]

        epochs = []
        for sess in sessions:
            epochs.append(get_epoch_from_path(sess))

        file_path_new = os.path.join(file_path, "session_" + str(max(epochs) + 1))
        os.mkdir(file_path_new)
    else:
        epoch_offset = get_epoch_from_path(file_path)
        file_path_new = os.path.dirname(file_path)

    # Allow dynamically growing GPU memory to avoid crashes
    config = tf.ConfigProto()
    config.gpu_options.allow_growth = True
    sess = tf.Session(config=config)
    set_session(sess)  # set this TensorFlow session as the default session for Keras

    # Set up generators that sequentially load training/validation data into memory
    training_generator = GraspDataGenerator(databases["training"], **generator_params)
    good_grasp_ratio = training_generator.get_good_grasp_ratio()
    generator_params["sample_class"] = -1  # deactivate up-/downsampling for validation
    generator_params["shuffle"] = False  # no need to shuffle validation data
    validation_generator = GraspDataGenerator(databases["validation"], **generator_params)

    # Retrieve the DNN model
    image_input_shape = tuple(generator_params["depth_image_dim"])
    grasp_data_input_shape = tuple(generator_params["grasp_data_dim"])
    num_outputs, model = get_dnn_model(image_input_shape, grasp_data_input_shape)
    training_generator.num_outputs = num_outputs
    validation_generator.num_outputs = num_outputs
    print(model.summary())
    print("Good grasps ratio: {:.4f}".format(good_grasp_ratio))
    print("Start learning rate: {:.6f}".format(learning_rate))

    # Compile model
    # opt = optimizers.SGD(lr=learning_rate, momentum=0.9, decay=0.0, nesterov=True)
    opt = optimizers.adam(lr=learning_rate)

    if num_outputs == 1:
        # model.compile(loss=binary_focal_loss(gamma=1.0, alpha=1-good_grasp_ratio), optimizer=opt, metrics=["accuracy"])  remove class_weights from fit!
        model.compile(loss="binary_crossentropy", optimizer=opt, metrics=["accuracy"])
    else:
        model.compile(loss="categorical_crossentropy", optimizer=opt, metrics=["accuracy"])

    # Apply weights of loaded model
    if not new_session:
        model_loaded = load_model(file_path, custom_objects={"binary_focal_loss_fixed": binary_focal_loss(gamma=1.0, alpha=1-good_grasp_ratio)})
        model.set_weights(model_loaded.get_weights())
        del model_loaded

    # Set up callbacks used for training
    full_file = os.path.join(file_path_new, "model_{epoch:03d}.h5")  # find a unique file name for saving
    cmc = ConditionalModelCheckpoint(full_file, verbose=1, start_epoch=train_params["start_saving_epoch"])
    logger = EpochLogger(training_generator.__len__())
    writer = CSVLogger(os.path.join(file_path_new, "history.csv"), separator=",", append=True)

    # Fit model and save resulting history for later processing
    # For binary_focal_loss
    # model.fit_generator(generator=training_generator, validation_data=validation_generator,
    #                    epochs=train_params["final_epoch"], callbacks=[cmc, logger, writer],
    #                    initial_epoch=epoch_offset, verbose=0, use_multiprocessing=True, workers=6)

    # For crossentropy loss
    model.fit_generator(generator=training_generator, validation_data=validation_generator, class_weight=[good_grasp_ratio, 1-good_grasp_ratio],
                        epochs=train_params["final_epoch"], callbacks=[cmc, logger, writer],
                        initial_epoch=epoch_offset, verbose=0, use_multiprocessing=True, workers=6)

# Retrieve the epoch that is part of each model file
def get_epoch_from_path(file_path):
    return int(os.path.basename(file_path).split("_")[-1].split(".")[0])

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

# Save all models after start epoch has been reached
class ConditionalModelCheckpoint(Callback):

    def __init__(self, filepath, verbose=0, start_epoch=1):
        super(ConditionalModelCheckpoint, self).__init__()
        self.filepath = filepath
        self.verbose = verbose
        self.start_epoch = max(start_epoch, 1)  # force between [1, inf)

    def on_epoch_end(self, epoch, logs=None):
        current_epoch = epoch + 1
        if current_epoch >= self.start_epoch:

            filepath = self.filepath.format(epoch=current_epoch, **logs)

            if self.verbose > 0:
                print("Epoch {}: Saving model to {}.".format(current_epoch, filepath))

            self.model.save(filepath, overwrite=True)

# Logs current progress only once per epoch. Should be the last callback in the callbacks list.
class EpochLogger(Callback):

    def __init__(self, batches_per_epoch=0):
        super(EpochLogger, self).__init__()
        self.batches_per_epoch = batches_per_epoch

    def on_train_begin(self, logs=None):
        self.final_epoch = self.params["epochs"]
        if self.batches_per_epoch > 0:
            print("Training has started with {} batches per epoch.".format(self.batches_per_epoch))
        else:
            print("Training has started.")

    def on_epoch_begin(self, epoch, logs=None):
        self.start_time = time()

    def on_epoch_end(self, epoch, logs=None):
        metrics = []
        search = ["loss", "val_loss"]
        for k in search:
            if k in logs:
                metrics.append(logs[k])

        duration = (time() - self.start_time)
        unit = "seconds"

        if duration >= 3600.0:
            duration /= 3600.0
            unit = "hours"
        elif duration >= 60.0:
            duration /= 60.0
            unit = "minutes"

        print("Epoch {}/{} - loss: {:.6f}, val loss: {:.6f}, duration: {:.4f} {}"
              .format(epoch+1, self.final_epoch, metrics[0], metrics[1], duration, unit))
