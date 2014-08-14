label=tau_measure
stamp=$(date '+%Y-%m-%d_%H-%M')
prefix="${label}_${stamp}"
bag="${prefix}.bag"
param="${prefix}.rosparam.yaml"

rosbag record -j -O $bag \
	/r_foot \
	/r_shank \
	/r_thigh \
	/l_foot \
	/l_shank \
	/l_thigh \
	/torso

rosparam dump $param &

wait
