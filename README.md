## XGO Mini 2 - ROS Humble Controller

This is the ROS2 Humble port of the [official ROS1 controller](https://www.yuque.com/luwudynamics/en/yhrwlm5mdu1trv3h) provided by the XGO team.

![image](https://github.com/sskorol/xgo_ros/assets/6638780/5b3d2315-8a3c-4c19-85e8-5fe14f2bc4da)

### Installation

It's quite tricky with lots of hacks. A big ToDo is to automate and simplify this process. Note that some steps might be inaccurate at this point.
So you can create PRs with fixes.

#### Base image

Flash Ubuntu server 22.04 via [RPi-imager](https://www.raspberrypi.com/software/)

#### Serial ports and BT configuration

Adjust `config.txt`:
```shell
sudo nano /boot/firmware/config.txt
```

- Ensure `enable_uart=1` is set to enable the UART interface.
- Add `dtoverlay=miniuart-bt` to ensure Bluetooth uses `ttyS0` and leaves `ttyAMA0` for general use.

Adjust `cmdline.txt`:
```shell
sudo nano /boot/firmware/cmdline.txt
```

Remove the section `console=serial0,115200`.

#### Serial console

To avoid serial conflicts, disable the console:
```shell
sudo systemctl stop serial-getty@ttyS0.service
sudo systemctl disable serial-getty@ttyS0.service
```

#### Permissions

Add current user to `dialout` and `tty` groups:
```shell
sudo usermod -a -G dialout,tty $USER
```

Set udev rules for ttyS0 and ttyAMA0:
```shell
sudo nano /etc/udev/rules.d/99-serial.rules
```

Add the following lines to it:
```text
KERNEL=="ttyAMA0", GROUP="tty,dialout", MODE="0666"
KERNEL=="ttyS0", GROUP="tty,dialout", MODE="0666"
```

Reload the udev rules:
```shell
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Reboot the Raspberry Pi to ensure all changes are applied:

```shell
sudo reboot
```

After rebooting, you should be able to use `ttyAMA0` for serial communication and `ttyS0` for Bluetooth without any permission issues.
These steps are required to make XGO accept serial commands and control the robot via the BT joystick.
Note that assigning BT to `ttyS0`` makes it less stable. And sometimes, you may see random disconnects.
But it's better than sacrificing the main serial channel to the less stable port.

### Additional dependencies

Note that I haven't tried to replicate every step from scratch yet, so there's a chance I missed some system libraries.
But at least the following one have to be installed:
```shell
sudo apt update && sudo apt install -y libjsoncpp-dev
```

### ROS2 Humble

Follow the [official installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) to set up Humble (base version).
Or you can try automated scripts like [this](https://github.com/Tiryoh/ros2_setup_scripts_ubuntu/blob/main/ros2-humble-ros-base-main.sh).

### Serial communication

ROS2 doesn't support the serial communication library used in lots of ROS1 controllers. So you have to build its catkin-less port manually:
```shell
git clone https://github.com/wjwwood/cxx_serial.git && cd cxx_serial
nano Makefile
```

A hardcoded `CMAKE_INSTALL_PREFIX` refers to `/tmp` dir. To avoid losing the build output after reboot, you have to update a path:
```text
CMAKE_FLAGS := -DCMAKE_INSTALL_PREFIX=/usr/local
```

Then build and install:
```shell
make && sudo make install
```

### XGO Mini2 controller

Get sources:
```shell
mkdir -p xgo_ws/src && cd xgo_ws/src && git clone -b humble https://github.com/sskorol/xgo_ros.git && cd ../
```

Ensure you've set up colcon and initialized rosdep:
```shell
sudo rosdep init
rosdep update
```

Install dependencies:
```shell
rosdep install --from-paths src --ignore-src --rosdistro=humble -r -y
```

Build XGO controller:
```shell
colcon build --symlink-install
# Add the following to .zshrc or .bashrc to persist changes
source install/local_setup.zsh # or *.bash, depending on your defaults
```

Run controller:
```shell
ros2 launch xgo2_ros xgo_control_launch.py
```

For a quick keyboard teleoperation test, run the following commands:
```shell
sudo apt install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### LD-X series LiDAR

Note that the existing implementation supports LD-X LiDAR launch. So, if you have one connected to the USB-C port, feel free to launch it via `use_lidar` flag.
I found the following [driver](https://github.com/Myzhar/ldrobot-lidar-ros2) one of the most professional. However, there's a known [bug](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2/issues/4) in all the existing implementations. It appears when you try running slam-toolbox. It seems like there's already a [workaround](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2/issues/4#issuecomment-1741854007) exists. I haven't tried it yet, though.

### Known issues and ToDos

- None of the existing teleop nodes allow you to control an arm and body. So, I created a separate node for that. It will be published soon. So stay tuned.
- I also adjusted the visualization by migrating a messy URDF to Xacro format. It still requires some polishing, though.
- Integrate more serial commands like speed control, moving mode, etc. Some of them mentioned in the [official docs](https://www.yuque.com/luwudynamics/en/acdzm2yqrkml35m7) didn't work for me. I need to contact the owners to get more details.
- Investigate the real limits of all the joints. Sometimes, I can bring XGO to an unrecoverable state. Velocity commands stop working. A robot just stuck in some pose.
It might be a partial "deadlock" (some joints might still work).
- Investigate and reproduce system crashes that prevent XGO from shutting down. SSH goes down, and the power button doesn't react on pressing. In this state, we should only wait until the battery dies.
- Find options for a more stable power supply that prevents the battery from draining when the external power supply is connected in active development mode.
Unfortunately, XGO can't work forever. And it makes the whole development experience awful.
- IMU data doesn't provide Gyro readings. I'm still confused if it hasn't been implemented yet or if a sensor doesn't support it. On the other hand, Z-readings always give me a constant error (a value is increasing with a constant speed). When I tried to visualize it in RViz, a robot turned around its Z-axis. I'm unsure if it's a sensor issue, magnetic fields' impact, or some other problem. I need to figure that out, as I can't fully rely on IMU data.
- Couldn't make slam-toolbox work due to a mentioned LiDAR issue. I'm going to apply a given patch to see if it helps.
- I still can't make it work with CHAMP. The biggest issue here is in the joint states. By default, this topic contains legs and arm data. However, CHAMP relies on leg data only. It crashes. I've already done some experiments (commenting arm joints / extending CHAMP state estimator). But no luck yet. ToDo: publish gaits, joints, and links based on the XGO characteristics provided to me by its team. I'd likely include these configs in a separate repo with the visualization part, which should run on more powerful hardware than RPi.
- I can't fully make XGO work in Gazebo yet for the same reason. We must split the arm and legs and rely on MoveIt configs (probably) at the end. I'm unsure about the best control plugins for both cases. Note about a great job by [MiniPupper team](https://github.com/mangdangroboticsclub/mini_pupper_ros) to support ROS2 for their dog. So, I use their code as inspiration in my experiments with XGO.
- Figure out the correct way of publishing odometry data for quadruped robots. The current implementation is just a workaround to build a full tf-tree required for the other steps to SLAM/Nav.
- Polish the code.
