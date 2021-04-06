#!/bin/bash 

if [ ! -d "~/map_save" ]; then
	mkdir ~/map_save
fi

cd ~/map_save

if [ "$1" != "-f" ]; then 
	echo "need name parameter. please use -f 'FILENAME'"
	cd ~ 
	exit 1
fi

if [ "$1" == "-f" ]; then
	echo "map saving... occ: 90, 80, 70, free: 10, 20, 30"
	rosrun map_server map_saver -f $2+9010 --occ 90 --free 10
	rosrun map_server map_saver -f $2+8020 --occ 80 --free 20
	rosrun map_server map_saver -f $2+7030 --occ 70 --free 30
	echo "map saved.";
	exit 1;
fi

cd ~

exit 1
