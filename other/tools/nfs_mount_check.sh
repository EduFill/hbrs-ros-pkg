#!/bin/bash
mount_point='/dev/sda1'

#if df -t nfs | grep -q '/mnt/mountpoint$' == 0
if !(df | grep -q $mount_point)
      then
	# try to mount	
	echo 'mount' $mount_point
fi
