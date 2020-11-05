# ensure there are no ros packages
sudo apt-get remove --autoremove ros-*

# check for updates
sudo apt update

# ensure /etc/ros removal
sudo rm -rf /etc/ros/

# download useful tools
sudo apt install git openssh-server

# install the python3 libraries 
sudo apt install -y python3 python3-dev python3-pip build-essential

# Remove python2
sudo apt purge -y python2.7-minimal

# link python -> python3
sudo ln -s /usr/bin/python3 /usr/bin/python

# Same for pip
sudo ln -s /usr/bin/pip3 /usr/bin/pip

# install the ros dependencies
sudo -H pip3 install rosdep rospkg rosinstall_generator rosinstall wstool vcstools catkin_tools catkin_pkg

# initialize catkin build environment
sudo rosdep init && rosdep update

# create catkin workspace
mkdir -p ~/ros_catkin_ws/src && cd "$_/.."

# initialize catkin workspace (will show warning about Extending... ignore that)
catkin config --init -DCMAKE_BUILD_TYPE=Release --blacklist rqt_rviz rviz_plugin_tutorials librviz_tutorial --install

# generate a ros melodic install
rosinstall_generator desktop_full --rosdistro melodic --deps --tar > melodic-desktop-full.rosinstall

# initialize the install
wstool init -j8 src melodic-desktop-full.rosinstall

# setup environment and install dependencies
export ROS_PYTHON_VERSION=3

# install wxPython
pip install -U -f https://extras.wxpython.org/wxPython4/extras/linux/gtk3/ubuntu-18.04 wxPython

# create install_skip file
printf '#/bin/bash\nif [ $(whoami) != root ]; then\n    echo You must be root or use sudo to install packages.\n    return\nfi\n\nfor pkg in "$@"\ndo\n    echo "Installing $pkg"\n    sudo apt-get -my install $pkg >> install.log\ndone' > install_skip

# make file executable
chmod +x install_skip

# install python 3 packages
sudo ./install_skip `rosdep check --from-paths src --ignore-src | grep python | sed -e "s/^apt\t//g" | sed -z "s/\n/ /g" | sed -e "s/\<python\>/python3/g"`

# skip python 2 packages
rosdep install --from-paths src --ignore-src -y --skip-keys="`rosdep check --from-paths src --ignore-src | grep python | sed -e "s/^apt\t//g" | sed -z "s/\n/ /g"`"

# rename all old python links to python3
find . -type f -exec sed -i 's/\/usr\/bin\/env[ ]*\<python\>/\/usr\/bin\/env python3/g' {} +

# remove all depricated install-layout=deb arguments
find ./ -name 'python_distutils_install.sh' -exec sed -i 's/--install-layout=deb//g' {} \;

# install python-empy
sudo apt install -y python-empy

# install uuv-sim
cd ~/ros_catkin_ws/src
git clone https://github.com/uuvsimulator/uuv_simulator.git
cd ~/ros_catkin_ws

# build ros
catkin build

# export python path
export PYTHONPATH=/usr/lib/python3/dist-packages

# source setup
source ~/ros_catkin_ws/install/setup.bash
