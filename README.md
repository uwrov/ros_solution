# ros_solution (camera_test)
An example workspace for our training project

## How to use this repo
### Installation
1. Clone this repo using `git clone`
2. CD into the cloned directory and make everything using `catkin_make`
3. Put in the command `source devel/setup.sh` in __any__ terminal which will be using items from your created ROS package.

At this point you will have created a new package `wb_sol` and have updated your ROS management to include files from the new package.

### Running
1. CD into the directory `src/wb_sol/urdf`
2. Run these magic spells:

```
roslaunch gazebo_ros empty_world.launch

rosrun gazebo_ros spawn_model -file wb.urdf -urdf -model wheely_boi
```
This will spawn in the wheely_boi model into an empty gazebo world.

3. Launch the keyboard controller:
```
rosrun wb_sol key_in.py
```

4. If you want to run the camera controller, you can either:
  - [Give webcam access to your vm](https://askubuntu.com/questions/4875/how-can-i-use-my-webcam-with-ubuntu-running-in-virtualbox)
  - Or pull in video stream from an online source. I recommend using a camera streaming app ([android](https://play.google.com/store/apps/details?id=com.spynet.camon&hl=en_US)) ([apple](https://apps.apple.com/us/app/ip-camera-lite/id1013455241))
    - You'll want a video resolution of 640x480, with a bitrate of 512/64 kpbs
    
5. Running Commands
```
rosrun xacro xacro wheely_boi.xacro > wb.urdf
rosrun image_view image_view image:=/wheely_boi/camera1/image_raw

```
