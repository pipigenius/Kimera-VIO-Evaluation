if [[ $1 == "kitti" ]]; then
	DATA=KITTI0046
	YAML_FILE="test_kitti.yaml"
elif [[ $1 == "euroc" ]]; then
	DATA=V1_01_easy
	YAML_FILE="test_offline.yaml"
fi

PIPELINE=$2
mkdir -p $PWD/results/$DATA/$PIPELINE
cp $PWD/results/output_posesVIO.csv $PWD/results/$DATA/$PIPELINE/traj_es.csv
python $PWD/ros_evaluation.py $YAML_FILE -a --save_results --save_plots --save_boxplots

savepath=$3

mv $PWD/results/$DATA/$PIPELINE/ $savepath

# rm -rf $PWD/results