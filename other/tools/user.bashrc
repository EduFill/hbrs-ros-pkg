
###### display git branch in prompt 
function git_branch {
  git branch --no-color 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/(\1) /'
}

###### set color
PS1='${debian_chroot:+($debian_chroot)}\[\033[01;31m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00;32m\] $(git_branch)\[\033[00m\]\$ '


alias yb1='ssh -X myrobot-pc1'
alias yb2='ssh -X myrobot-pc2'


### ROS config
source /opt/ros/fuerte/setup.bash

export ROS_MASTER_URI=http://localhost:11311
export ROBOT=myrobot
export ROBOT_ENV=myenv

export ROS_PACKAGE_PATH=~/RoboCupAtWork:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=~/external_software:$ROS_PACKAGE_PATH

export ROSCONSOLE_FORMAT='[${severity}] [${node}]: ${message}'

### Status output
echo "ROS-Version:" $ROS_ROOT
echo "ROBOT:" $ROBOT
echo "ENV:" $ROBOT_ENV
echo "MASTER:" $ROS_MASTER_URI


