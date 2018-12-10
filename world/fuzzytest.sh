#!/bin/bash

worlds[0]=moon.world
worlds[1]=door.world
worlds[2]=cylinder.world
worlds[3]=horseshoe.world
worlds[4]=corner.world
worlds[5]=bluntangle.world

dist=5

for world in  ${worlds[*]}
do
	printf "   %s\n" $world
	for i in -1.57 -1.04 -0.79 -0.52 -0.17 0 0.17 0.52 0.79 1.04 1.57
	do
		xpos=$(echo "-$dist*c($i)" | bc -l)
		ypos=$(echo "-$dist*s($i)" | bc -l)
		yaw=$(echo "$i" | bc -l)
		line="      <pose>$xpos $ypos 0 0 0 $yaw<\\/pose>"
		#echo $line
		sed -i "37s/.*/$line/" $world
		for j in 1 2 3 4 5 6 7 8 9 10
		do
			echo Test $j for angle $i for world $world
			sh gazebo_server.sh $world &>/dev/null &
			../build-robot_control-Desktop-Debug/robot_control &>/dev/null &
			disown
			sleep 30s
			killall -9 gzserver
			killall -9 robot_control

                        mv distances.csv "../test_results/"$world"_"$i"_"$j"_dist.csv"
                        mv angles.csv "../test_results/"$world"_"$i"_"$j"_angle.csv"

		done
                paste -d ' ' "time.csv" "../test_results/"$world"_"$i"_"*"_dist.csv" > "../test_results/"$world"_"$i"_dist.csv"
                paste -d ' ' "time.csv" "../test_results/"$world"_"$i"_"*"_angle.csv" > "../test_results/"$world"_"$i"_angle.csv"

                rm time.csv
                rm "../test_results/"$world"_"$i"_"*"_dist.csv"
                rm "../test_results/"$world"_"$i"_"*"_angle.csv"
	done
done
