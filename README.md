# spark_vio_evaluation
Code to evaluate and tune SPARK VIO pipeline.

## Evaluation on the ROS wrapper 
Go to `evaluation_ros` directory 
```
cd spark_vio_evaluation/evaluation_ros
```
Note that evaulation is done with the EuRoC dataset. Specifically the V1_01 data. Make sure you have downloaded the rosbags. First, depending on if you want to evaluate the online or offline version do either 
```
./evaluate_offline.sh <path-to-V1_01_easy.bag>
```
For offline, or 
```
./evaluate_online.sh <path-to-V1_01_easy.bag>
```

Then run 
```
./evaluate.sh <path-to-desired-folder-to-save-in>
```
To evaluate and generate plots. 
