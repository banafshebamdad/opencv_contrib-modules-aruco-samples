#!/bin/bash

#
# Banafshe Bamdad
# Mo Jun 12 20.23.09
# This script get full path of a directory containing images and detect ArUco markers in the images
# Usage: bash banafshe_detect_marker.sh "/full/path/to/folder/containg/images/*" 
# e,g. bash banafshe_detect_marker.sh  "/home/banafshe/Documents/kimera/colmap_Winti_HB/frames_from_rosbag/*"
# Note: "/full/path/to/folder/containg/images/*" must be wrapped n " " and the path should be ended in *.
#

echo "Hello $1"
for file in $1
do
	echo "$file"
	./delete_me -v="$file" -d=0 -l=0.24 -r --refine=3
done
