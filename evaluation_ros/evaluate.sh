mkdir -p $PWD/results/V1_01_easy/S
cp $PWD/results/output_posesVIO.csv $PWD/results/V1_01_easy/S/traj_es.csv
python $PWD/ros_evaluation.py

savepath=$1

mv $PWD/results/V1_01_easy/S/ $savepath

rm -rf $PWD/results