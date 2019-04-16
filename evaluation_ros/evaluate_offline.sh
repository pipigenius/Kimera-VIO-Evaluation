if [ ! -d "results" ]; then
  mkdir results
fi
ROSBAG_DATA=$1 # the rosbag you want to evaluate on 
roslaunch spark_vio_ros spark_vio_ros_euroc_offline.launch data:=$ROSBAG_DATA LOG_OUTPUT:=True OUTPUT_PATH:=$PWD/results VISUALIZE:=False