label=tau_measure
stamp=$(date '+%Y-%m-%d_%H-%M')
prefix="${label}_${stamp}"
bag="${prefix}.bag"
param="${prefix}.rosparam.yaml"

rosbag record -j -O $bag \
	/human_under/data1 \
	/human_under/data2 \
	/human_under/data3 \
	/human_under/data4 \
	/tau_under

rosparam dump $param &

wait
