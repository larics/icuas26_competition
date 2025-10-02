#!/bin/bash
function cleanup() {
	pkill -x cf2
	pkill -9 ruby
}

function spawn_model() {
	MODEL=$1
	N=$2 # Cf ID
	X=$3 # spawn x position
	Y=$4 # spawn y position
	X=${X:=$X}
	Y=${Y:=$Y}
	SUPPORTED_MODELS=("crazyflie", "crazyflie_thrust_upgrade")
	if [[ " ${SUPPORTED_MODELS[*]} " != *"$MODEL"* ]];
	then
		echo "ERROR: Currently only vehicle model $MODEL is supported!"
		echo "       Supported Models: [${SUPPORTED_MODELS[@]}]"
		trap "cleanup" SIGINT SIGTERM EXIT
		exit 1
	fi
	
	working_dir="$build_path/$n"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

	pushd "$working_dir" &>/dev/null


	set --
	set -- ${@} ${model_path}/tools/crazyflie-simulation/simulator_files/gazebo/launch/jinja_gen.py
	set -- ${@} ${model_path}/tools/crazyflie-simulation/simulator_files/gazebo/models/${MODEL}/model.sdf.jinja
	set -- ${@} ${model_path}/tools/crazyflie-simulation/simulator_files/gazebo
	set -- ${@} --cffirm_udp_port $((19950+${N}))
	set -- ${@} --cflib_udp_port $((19850+${N}))
	set -- ${@} --cf_id $((${N}))
	set -- ${@} --cf_name cf
	set -- ${@} --output-file /tmp/${MODEL}_${N}.sdf

	python3 ${@}

	echo "Spawning ${MODEL}_${N} at ${X} ${Y}"

    gz service -s /world/${world}/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 300 --req 'sdf_filename: "/tmp/'${MODEL}_${N}'.sdf", pose: {position: {x:'${X}', y:'${Y}', z: 0.5}}, name: "'${MODEL}_${N}'", allow_renaming: 1'
	
	echo "starting instance $N in $(pwd)"
	$build_path/cf2 $((19950+${N})) > out.log 2> error.log &

	popd &>/dev/null
}

if [ "$1" == "-h" ] || [ "$1" == "--help" ]
then
	echo "Description: This script is used to spawn multiple vehicles in gazebo from a coordinates list text file."
	echo "Usage: $0 [-m <vehicle_model>] [-w <world>] [-f <file_name>]"
	exit 1
fi

while getopts m:w:f:n: option
do
	case "${option}"
	in
		m) VEHICLE_MODEL=${OPTARG};;
		w) WORLD=${OPTARG};;
		f) COORDINATES_FILE=${OPTARG};;
		n) NUMBER_OF_ROBOTS=${OPTARG};;
	esac
done

world=${WORLD:=crazysim_default}
vehicle_model=${VEHICLE_MODEL:="crazyflie"}
coordinates_file=${COORDINATES_FILE:="single_origin.txt"}
number_of_robots=${NUMBER_OF_ROBOTS:=5}
export CF2_SIM_MODEL=gz_${vehicle_model}

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/.."
echo "$src_path"
model_path="/root/CrazySim/crazyflie-firmware"

build_path=${model_path}/sitl_make/build


echo "killing running crazyflie firmware instances"
pkill -x cf2 || true

sleep 1

source ${src_path}/launch/setup_gz.bash ${model_path} ${build_path} ${src_path}

echo "Starting gazebo"
gz sim -s -r ${src_path}/worlds/${world}.sdf -v 3 &
sleep 3

n=0
while IFS= read -r line || [ -n "$line" ] && [ "$n" -lt "$number_of_robots" ];do
	fields=($(printf "%s" "$line"|cut -d',' --output-delimiter=' ' -f1-))
	spawn_model ${vehicle_model} $(($n)) ${fields[0]} ${fields[1]}
	n=$(($n + 1))
done < "$SCRIPT_DIR/drone_spawn_list/${coordinates_file}"

trap "cleanup" SIGINT SIGTERM EXIT

echo "Starting gazebo gui"
#gdb ruby
gz sim -g