#!/bin/bash

# declare -a arr=("/home/allevato/catkin_ws/src/kinova_ros")

declare -a arr=$(ls -d */)

## now loop through the above array
# for i in "${arr[@]}"
for i in $(ls -d */)
do
  echo "=========================================================="
  echo "$i"
  cd $i
  git status
  cd ../
  
  # or do whatever with individual element of the array
done
