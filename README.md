## XGO Mini 2 - ROS Humble Controller

This is the ROS2 Humble port of the [official ROS1 controller](https://www.yuque.com/luwudynamics/en/yhrwlm5mdu1trv3h) provided by XGO team.

### Installation

It's quite tricky with lots of hacks. A big ToDo is to automate and simplify this process. Note that some steps might be inaccurate at this point.
So feel free to create PRs with fixes.

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

To avoid serial conflicts, disable console:
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

After rebooting, you should be able to use `ttyAMA0` for serial communication and `ttyS0` for Bluetooth, without any permission issues.
These steps are requried to make XGO accept serial commands and to control the robot via BT joystick.
Note that assigning BT to `ttyS0`` makes it less stable. And sometimes you may see random disconnects.
But it's better than sacrificing the main serial channel to the less stable port.

### Additional dependencies

Note that I haven't tried to replicate every step from scratch yet, so there's a chance I missed some system libraries.
But at least the following one have to be installed:
```shell
sudo apt update && sudo apt install -y libjsoncpp-dev
```

### ROS2 Humble

Follow the [official installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) to set up Humble (base version).
Or you can try some automated scripts like [this](https://github.com/Tiryoh/ros2_setup_scripts_ubuntu/blob/main/ros2-humble-ros-base-main.sh).

### Serial communication

ROS2 doesn't support serial communication library used in lots of ROS1 controllers anymore. So you have to build its catkin-less port manually:
```shell
git clone https://github.com/wjwwood/cxx_serial.git && cd cxx_serial
nano Makefile
```

There's a hardcoded `CMAKE_INSTALL_PREFIX` that refers to `/tmp` dir. To avoid loosing the build output after reboot, you have to update a path:
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
mkdir -p xgo_ws/src && cd xgo_ws/src && git clone -b humble https://github.com/sskorol/xgo-mini.git && cd ../
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

Note that the existing implementation supports LD-X LiDAR launch. So if you have one connected to the USB-C port, feel free to launch it via `use_lidar` flag.
I found the following [driver](https://github.com/Myzhar/ldrobot-lidar-ros2) one of the most professional at this point. However, note that there's a know [bug](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2/issues/4) in all the existing implementations. It appears when you try running slam-toolbox. Seems like there's already a [workaround](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2/issues/4#issuecomment-1741854007) exists. Haven't tried it yet, though.

### Known issues and ToDos

- None of the existing teleop nodes allows you controlling an arm along with the body. So I created a separate node for that. Will be published soon. So stay tuned.
- I also made adjustments to visualization part by migrating an existing messy URDF to Xacro format. It still requries some polishing though.
- Integrate more serial commands like speed control, moving mode, etc. Note that some of them mentioned in the [official docs](https://www.yuque.com/luwudynamics/en/acdzm2yqrkml35m7) didn't work for me. Need to contact owners to get more details.
- Investigate real limits of all the joints. Sometimes I can bring XGO to unrecoverable state. Velocity commands stop working. A robot just stuck at some pose.
Note that it might be a partial "deadlock" (some joints might still work).
- Investigate and reproduce system crash that prevents XGO from shutting down. SSH goes down, power button doesn't react on pressing. In this state we should only wait until the battery dies.
- Find options for more stable power supply that prevents battery from draining when external power supply is connected in active development mode.
Unfortunately, XGO can't work forever. And it make the whole development experience aweful.
- IMU data doesn't provide Gyro readings. I'm still a little bit confused if it's hasn't been implemented yet, or a sensor doesn't support it. On the other hand, Z-readings always give me a constant error (a value is increasing with a constant speed). When I tried to visualize it in RViz, a robot was turning around its Z-axis. Not yet sure if it's a sensor issue or magnetic fields impact, or some other problem. Need to figure that out, as I can't fully reply on IMU data at this point.
- Couldn't make slam-toolbox work due to a mentioned above LiDAR issue. Going to apply a given patch to see if it helps.
- Still can't make it work with CHAMP. The biggest issue here is in the joint states. By default, this topic contains legs and arm data. However, CHAMP replies on legs only. It crashes. Already performed some experiments. But on luck yet. ToDo: publish gaits, joints, and links based on the XGO characteristics provided to me by its team. Would likely include these configs into a separate repo with visualization part which should be running on more powerful hardware than RPi.
- Can't fully make XGO work in Gazebo yet due to the same reason as above. Seems like we need to split the arm and legs, and rely on MoveIt configs at the end. Not sure about the best control plugins for both cases though. Note about a great job done by [MiniPupper team](https://github.com/mangdangroboticsclub/mini_pupper_ros) to support ROS2 for their dog. So I use there code as inspiration in my experiments with XGO.
- Figure out the correct way of publishing odometry data for quadrupted robots. Current implementation is just a workaround to build a full tf-tree required for the other steps on the way to SLAM/Nav.
- Polish the code.