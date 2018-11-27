#!/bin/bash

world=door.world

for i in 1 2 3 4 5
do
	sh gazebo_server.sh $world &
	sleep 10s
	killall -9 gzserver
done

echo done
