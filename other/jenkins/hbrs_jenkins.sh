#!/bin/bash

# get the name of ROSRELEASE
RELEASE=$1

echo ""
echo "-------------------------------------------------------"
echo "==> RELEASE =" $RELEASE
echo "-------------------------------------------------------"
echo ""


# installing ROS release
echo -e "\n\n##################################################"
echo "apt autoclean, update, upgrade and dist-upgrade"
echo "##################################################"
sudo apt-get autoclean
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get dist-upgrade -y

echo -e "\n\n##################################################"
echo "install python-setuptools, rosinstall and vcstools"
echo "##################################################"
sudo apt-get install python-setuptools -y
sudo easy_install -U rosinstall vcstools

echo -e "\n\n##################################################"
echo "execute repository.debs script"
echo "##################################################"
$WORKSPACE/repository.debs -y

#echo -e "\n\n##################################################"
#echo "ros-$RELEASE-*"
#echo "##################################################"
#sudo apt-get install ros-$RELEASE-* -y

#echo -e "\n\n##################################################"
#echo "remove moveit packages in fuerte"
#echo "##################################################"
#sudo apt-get remove ros-fuerte-moveit-core  ros-fuerte-moveit-msgs -y

echo -e "\n\n##################################################"
echo "autoremove"
echo "##################################################"
sudo apt-get autoremove -y

# setup ROS environment
. /opt/ros/$RELEASE/setup.bash

# execute repository.rosinstall of each repository
echo -e "\n\n##################################################"
echo "rosinstall"
echo "##################################################"
rm -rf $WORKSPACE/../ext_pkgs/.rosinstall
rosinstall $WORKSPACE/../ext_pkgs /opt/ros/$RELEASE $WORKSPACE/repository.rosinstall --delete-changed-uris --rosdep-yes

# define amount of ros prozesses during build for multi-prozessor machines
#COUNT=$(cat /proc/cpuinfo | grep 'processor' | wc -l)
#COUNT=$(echo "$COUNT*2" | bc)
export ROS_PARALLEL_JOBS=-j1

# add whole directory of the job the the ROS package path
export ROS_PACKAGE_PATH=$WORKSPACE/..:$ROS_PACKAGE_PATH

echo ""
echo "-------------------------------------------------------"
echo "==> RELEASE =" $RELEASE
echo "==> WORKSPACE =" $WORKSPACE
echo "==> ROS_ROOT =" $ROS_ROOT
echo "==> ROS_PACKAGE_PATH =" $ROS_PACKAGE_PATH
echo "-------------------------------------------------------"
echo ""

# installing dependencies and building
export USE_NORMAL_SUDO=1    # for youbot_oodl
cd $WORKSPACE
rosdep install * -y
rosmake -r * --skip-blacklist --profile --status-rate=1

# check if building is succesfull, otherwise don't perform test and exit
if [ $? != "0" ]; then
	echo "rosmake failed, skipping tests"
	exit 1
fi
