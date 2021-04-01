# This file is part of Klampt_grasp_simulator.
# Copyright (C) 2021  Marc Riedlinger

# Klampt_grasp_simulator is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

#!/bin/bash

data_directory=../data/objects/test
input_ext=.off
output_ext=.obj

for file in $(find $data_directory -name "*$input_ext")
do
	filename=$(basename $file)
	filename=${filename%.*}
	output=$data_directory/$filename$output_ext
	echo $output
	printf "mesh \"$filename$input_ext\"\nT 1 0 0 0 1 0 0 0 1      0 0 0\nmass 0.1\nautoMass\nkStiffness 10000\nkDamping 3000\nkRestitution 0\nkFriction 0.5" > $output
done
