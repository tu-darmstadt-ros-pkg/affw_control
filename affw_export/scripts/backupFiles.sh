#!/bin/sh

if [ -d /tmp/affw_data ]; then
	echo Backing up
	d=$(date +%Y-%m-%d_%H-%M-%S)
	dir="affw_data_$d"
	mv /tmp/affw_data /tmp/$dir
fi
