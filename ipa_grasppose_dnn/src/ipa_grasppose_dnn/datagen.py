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
"""
Resource: https://stanford.edu/~shervine/blog/keras-how-to-generate-data-on-the-fly
Special thanks to the authors for providing the framework

This class is responsible for loading the processed grasp database in batches into the neural network
"""

import numpy as np
from numpy.random import RandomState
import keras
import cv2
import os


class GraspDataGenerator(keras.utils.Sequence):
    object_subfolders = ["bad", "good"]  # subfolders that hold good or bad grasp poses

    # Generates data for Keras
    def __init__(self, database_path, depth_image_dim=[64, 64, 1], grasp_data_dim=[8, 8, 4],
                 batch_size=32, sample_class=1, sample_ratio=1.0, sample_seed=0, shuffle=True):

        # Initialization of other instance variables
        self.object_names = []
        self.files_per_object = [np.array([], dtype=int), np.array([], dtype=int)]  # file count for good/bad grasps
        self.database_path = database_path
        dim = list(depth_image_dim)  # take a copy
        dim.insert(0, batch_size)
        self.depth_data_shape = tuple(dim)
        dim = list(grasp_data_dim)
        dim.insert(0, batch_size)
        self.grasp_data_shape = tuple(dim)
        self.batch_size = batch_size
        self.shuffle = shuffle
        self.num_outputs = 1  # is set later on

        # Get object list from database root folder
        object_list = [obj for obj in os.listdir(database_path)
                       if os.path.isdir(os.path.join(database_path, obj))]

        # Check if given object_list is valid
        for obj in object_list:
            folder_path = os.path.join(database_path, obj)
            bad_path = os.path.join(folder_path, self.object_subfolders[0])
            good_path = os.path.join(folder_path, self.object_subfolders[1])
            if not os.path.isdir(good_path) or not os.path.isdir(bad_path):
                print("Folder for "+folder_path+" does not have good/bad subfolders, skipping.")
            else:
                self.object_names.append(obj)
                # count datasets inside good folder
                file_count = len(os.listdir(good_path))
                self.files_per_object[1] = np.append(self.files_per_object[1], file_count)
                # count datasets inside bad folder
                file_count = len(os.listdir(bad_path))
                self.files_per_object[0] = np.append(self.files_per_object[0], file_count)

        # Create a huge indexes list containing the necessary files
        prng_good = RandomState(sample_seed)  # provide reproducibility
        prng_bad = RandomState(sample_seed)
        good_files = np.sum(self.files_per_object[1], dtype=int)
        good_indexes = np.arange(1, good_files+1)  # file indexes as they lie in the subfolders, not 0-indexed
        prng_good.shuffle(good_indexes)
        bad_files = np.sum(self.files_per_object[0], dtype=int)
        bad_indexes = np.arange(1, bad_files+1)
        prng_bad.shuffle(bad_indexes)
        if sample_class == 0:  # sample class 0 -> bad grasps
            bad_size = int(round(good_files * sample_ratio))  # sample bad files so we have bad_size files in the end
            self.final_size = good_files + bad_size
            self.good_grasp_ratio = good_files.astype(float) / self.final_size
            self.indexes = [np.empty((self.final_size,), dtype=np.int8),
                            np.empty((self.final_size,), dtype=np.int64)]  # [labels, indexes]

            for idx in xrange(self.final_size):  # create full list of file indexes and corresponding labels
                if idx < bad_size:  # bad grasps
                    self.indexes[0][idx] = 0  # label so we find the correct subfolder later on
                    self.indexes[1][idx] = bad_indexes[idx % bad_files]
                else:  # good grasps
                    self.indexes[0][idx] = 1
                    self.indexes[1][idx] = good_indexes[(idx - bad_size) % good_files]
        elif sample_class == 1:  # sample class 1 -> good grasps
            good_size = int(round(bad_files * sample_ratio))
            self.final_size = bad_files + good_size
            self.good_grasp_ratio = float(good_size) / self.final_size
            self.indexes = [np.empty((self.final_size,), dtype=np.int8),
                            np.empty((self.final_size,), dtype=np.int64)]

            for idx in xrange(self.final_size):
                if idx < good_size:  # good grasps
                    self.indexes[0][idx] = 1
                    self.indexes[1][idx] = good_indexes[idx % good_files]
                else:  # bad grasps
                    self.indexes[0][idx] = 0
                    self.indexes[1][idx] = bad_indexes[(idx - good_size) % bad_files]
        else:  # do not perform any sampling, keep original ratio
            self.final_size = bad_files + good_files
            self.good_grasp_ratio = good_files.astype(float) / self.final_size
            self.indexes = [np.empty((self.final_size,), dtype=np.int8),
                            np.empty((self.final_size,), dtype=np.int64)]

            for idx in xrange(self.final_size):  # create full list of file indexes and corresponding labels
                if idx < good_files:  # good grasps
                    self.indexes[0][idx] = 1
                    self.indexes[1][idx] = good_indexes[idx % good_files]
                else:  # bad grasps
                    self.indexes[0][idx] = 0
                    self.indexes[1][idx] = bad_indexes[(idx - good_files) % bad_files]

        # Shuffle index arrays once so good files and bad files won't lie one after another anymore
        self.shuffle_indexes()

    def shuffle_indexes(self):
        rng_state = np.random.get_state()
        np.random.shuffle(self.indexes[0])
        np.random.set_state(rng_state)  # ensure both arrays are shuffled the same way
        np.random.shuffle(self.indexes[1])

    def get_classes(self):
        elements = self.__len__()*self.batch_size
        return self.indexes[0][0:elements]  # returns np.array of type np.int8

    def get_good_grasp_ratio(self):  # ratio of label 1
        return self.good_grasp_ratio

    def __len__(self):
        # Denotes the number of batches per epoch
        return int(np.floor(self.final_size / self.batch_size))

    def __getitem__(self, index):
        # Generate one batch of data
        # index defines the current batch, it starts at 0 and counts upwards until __len__
        indexes = range(index*self.batch_size, (index+1)*self.batch_size)

        # Find file that corresponds to each mapped index
        final_dataset_list = []
        for current_idx in indexes:
            current_class = self.indexes[0][current_idx]  # 1: good grasp, 0: bad grasp
            current_file = self.indexes[1][current_idx]  # store current file index
            total_file_count = 0
            for i, file_count in enumerate(self.files_per_object[current_class]):
                total_file_count += file_count

                # for first time current_file is smaller-equal -> file belongs to current object
                if current_file <= total_file_count:
                    # index of dataset relative to its object folder
                    relative_file_index = current_file - total_file_count + file_count
                    object_name = self.object_names[i]
                    subfolder = self.object_subfolders[current_class]
                    file_name = object_name+"_dataset_"+str(relative_file_index)+".yml"
                    file_path_relative = os.path.join(object_name, subfolder, file_name)  # relative to database_path
                    final_dataset_list.append(file_path_relative)
                    break

        # Generate data
        X, y = self.__data_generation(final_dataset_list)

        return X, y

    def on_epoch_end(self):
        # Updates indexes after each epoch
        if self.shuffle:
            self.shuffle_indexes()

    def __data_generation(self, final_dataset_list):
        # Generates data containing batch_size samples X : (n_samples, *dim, n_channels)
        # Initialization
        X = [np.empty(self.depth_data_shape), np.empty(self.grasp_data_shape)]
        y = np.empty(self.batch_size, dtype=int)

        # Generate data
        for i, file_path in enumerate(final_dataset_list):  # iterate over batches
            fs_read = cv2.FileStorage(os.path.join(self.database_path, file_path), cv2.FILE_STORAGE_READ)
            if fs_read.isOpened():
                depth_patch = fs_read.getNode("depth_patch").mat()  # mat returns numpy.ndarray (64x64)
                # depth_patch = cv2.resize(depth_patch, (32,32), interpolation=cv2.INTER_AREA) # downscale to 32x32

                # only load first grasp pose for this work
                # grasp_pose = fs_read.getNode("grasp_pose").mat()[0, 0:self.grasp_data_shape[3]]  # full pose
                grasp_pose = fs_read.getNode("grasp_pose").mat()[0, 3:(3+self.grasp_data_shape[3])]  # only rotation
                grasp_pose_resized = np.empty(self.grasp_data_shape[1:4])
                for l in xrange(self.grasp_data_shape[3]):
                    grasp_pose_resized[:, :, l] = np.full(self.grasp_data_shape[1:3], grasp_pose[l])
                label = fs_read.getNode("grasp_label").mat()[0, 0]  # only load corresponding label

                # Store data
                X[0][i, :, :, 0] = depth_patch
                X[1][i] = grasp_pose_resized

                # Store corresponding label
                y[i] = int(label)
            else:
                print("Error, could not load "+file_path+", skipping for training.")

        if self.num_outputs == 1:
            return X, y  # single output neuron
        else:
            return X, keras.utils.to_categorical(y, num_classes=self.num_outputs, dtype=int) # more output neurons
