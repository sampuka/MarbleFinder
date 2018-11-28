#!/bin/bash

worlds[0]=door.world
worlds[1]=moon.world
worlds[2]=cylender.world
worlds[3]=horseshoe.world

for world in  ${worlds[*]}
do
	printf "   %s\n" $world
	for i in 1 2 3 4
	do
		xpos=0
		ypos=$(echo "2*2" | bc -l)
		yaw=$(echo "2*2" | bc -l)
		line="      <pose>$xpos $ypos 0 0 0 $yaw<\\/pose>"
		sed -i "37s/.*/$line/" $world
		for j in 1 2 3 4 5 6 7 8 9 10
		do
			echo Test $j for angle $i for world $world
			sh gazebo_server.sh $world &>/dev/null &
			../build-robot_control-Desktop-Debug/robot_control &>/dev/null &
			disown
			sleep 1s
			killall -9 gzserver 
			killall -9 robot_control
			mv results.csv "../test_results/test_"$world"_"$i"_"$j".csv"
		done
	done
done
