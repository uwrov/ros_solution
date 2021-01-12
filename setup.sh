# ensure there are no ros packages
sudo apt-get remove --autoremove ros-*

# check for updates
sudo apt update

# ensure /etc/ros removal
sudo rm -rf /etc/ros/

# install the python3 libraries 
sudo apt install -y python3 python3-dev python3-pip build-essential

# Get catkin_tools
sudo pip3 install git+https://github.com/catkin/catkin_tools

# Remove python2
sudo apt purge -y python2.7-minimal

# link python -> python3
sudo ln -s /usr/bin/python3 /usr/bin/python

# Same for pip
sudo ln -s /usr/bin/pip3 /usr/bin/pip

# install the ros dependencies
sudo -H pip3 install rosdep rospkg rosinstall_generator rosinstall wstool vcstools catkin_tools catkin_pkg

echo "creating workspace directory"

# create catkin workspace
mkdir -p /home/pi/ros_catkin_ws/src && cd /home/pi/ros_catkin_ws

# initialize catkin build environment
sudo rosdep init && rosdep update
sudo catkin init .

# initialize catkin workspace (will show warning about Extending... ignore that)
catkin config --init -DCMAKE_BUILD_TYPE=Release --blacklist rqt_rviz rviz_plugin_tutorials librviz_tutorial --install

# generate a ros melodic install
rosinstall_generator ros_comm --rosdistro melodic --deps --tar > ros-melodic-ros-base.rosinstall

# initialize the install
wstool init -j8 src ros-melodic-ros-base.rosinstall

# setup environment and install dependencies
export ROS_PYTHON_VERSION=3

# create install_skip file
printf '#/bin/bash\nif [ $(whoami) != root ]; then\n    echo You must be root or use sudo to install packages.\n    return\nfi\n\nfor pkg in "$@"\ndo\n    echo "Installing $pkg"\n    sudo apt-get -my install $pkg >> install.log\ndone' > install_skip

# make file executable
chmod +x install_skip

# install python 3 packages
sudo ./install_skip `rosdep check --from-paths src --ignore-src | grep python | sed -e "s/^apt\t//g" | sed -z "s/\n/ /g" | sed -e "s/\<python\>/python3/g"`

# skip python 2 packages
rosdep install --from-paths src --ignore-src -y --skip-keys="`rosdep check --from-paths src --ignore-src | grep python | sed -e "s/^apt\t//g" | sed -z "s/\n/ /g"`"

# rename all old python links to python3
sudo sh -c 'find . -type f -exec sed -i 's/\/usr\/bin\/env[ ]*\<python\>/\/usr\/bin\/env python3/g' {} +'

# remove all depricated install-layout=deb arguments
sudo sh -c 'find ./ -name 'python_distutils_install.sh' -exec sed -i 's/--install-layout=deb//g' {} \;'

# install python-empy
sudo apt install -y python-empy

# install common msgs
cd /home/pi/ros_catkin_ws/src
git clone https://github.com/ros/common_msgs.git
cd common_msgs
git checkout jade-devel
cd /home/pi/ros_catkin_ws

# build ros
catkin build

# export python path
export PYTHONPATH=/usr/lib/python3/dist-packages

# source setup
source /home/pi/ros_catkin_ws/install/setup.bash

# isntall pigpio
cd /home/pi
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install
cd /home/pi
rm master.zip
