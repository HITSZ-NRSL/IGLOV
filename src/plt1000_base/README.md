PLT1000 base simulation

## 1. Path setting

add below command to ~/.bashrc, for gazebo model

```export GAZEBO_MODEL_PATH=$~/catkin_ws$/src/plt1000_base/plt_gazebo/model```


## 2. install libary
```sudo apt-get install ros-kinetic-velodyne ros-kinetic-velodyne-gazebo-plugins ros-kinetic-velodyne-laserscan``` 


## 3. Build
```catkin build plt_base plt_gazebo plt_control```

```source devel/setup.bash```

## 4. Run

Launch gazebo(different launch file refer to different scenes)

```roslaunch plt_base example.launch```

run command terminal

change the permission of robot_keyboard_teleop.py


```chmod +x src/plt1000_base/plt_control/src/robot_keyboard_teleop.py```


```rosrun plt_control robot_keyboard_teleop.py```

