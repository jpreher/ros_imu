label=tau_measure
stamp=$(date '+%Y-%m-%d_%H-%M')
prefix="${label}_${stamp}"
bag="${prefix}.bag"
param="${prefix}.rosparam.yaml"

rosbag record -j -O $bag \
	/proxi_hip/data1 

rosparam dump $param &

wait
