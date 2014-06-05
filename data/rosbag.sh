label=tau_measure
stamp=$(date '+%Y-%m-%d_%H-%M')
prefix="${label}_${stamp}"
bag="${prefix}.bag"
param="${prefix}.rosparam.yaml"

rosbag record -j -O $bag \
#	/right_leg/data1 \
#	/right_leg/data2 \
	/left_leg/data1 \
	/left_leg/data2 \
#	/left_leg/data3 \
#	/tau_under/left_shank_earth \
#	/tau_under/left_thigh_shank \
#	/tau_under/left_hip_thigh \
#	/tau_under/right_shank_earth \
#	/tau_under/right_thigh_shank \
#	/tau_under/right_hip_thigh \
#	/tau_under/tau \
#	/tau_under/right_is_stance \

rosparam dump $param &

wait
