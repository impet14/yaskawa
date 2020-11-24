# OpenZen Node for ROS

This software allows to forward sensor data from sensor connected via OpenZen to ROS.

OpenZen is a library for high performance sensor data streaming and processing and supports multiple sensor models: <https://bitbucket.org/lpresearch/openzen/>

## Requirements

### Tools & Compiler

To compile this software, at least G++ version 7.0 and cmake version 3.10 need to be installed. If your default compiler is an older version
you can install a recent GCC with your distributions package manager and provide the name of the GCC to the catkin_make command
like so:

```
catkin_make -DCMAKE_C_COMPILER=gcc-7 -DCMAKE_CXX_COMPILER=g++-7
```
### Serial Port Access Rights

After this call, you should logout and login with this user to ensure the changed permissions are in effect.

To allow access to sensors connected via USB, you need to ensure that the user running the ROS sensor node
has access to the /dev/ttyUSB devices. You can do this by adding the user to the dialout group.

```
sudo adduser <username> dialout
```

## Compilation

To compile this driver in your ROS setup, follow these steps:
```
mkdir -p catskin_ws/src
cd catskin_ws/src

git clone --recurse-submodules https://bitbucket.org/lpresearch/openzenros.git

# get your ROS environment going
source /opt/ros/melodic/setup.bash
cd ..
catkin_make
source ./devel/setup.bash
```
## Running the Driver

Open another terminal window and run the ROS core:

```
source /opt/ros/melodic/setup.bash
roscore
```

You can then run the OpenZen ROS driver with this command in the window
you used to compile the software:

```
rosrun openzen_sensor openzen_sensor_node
```

By default, it will connect to the first available sensor. If you want to connect to
a specific sensor, you can use the serial name of the sensor as parameter, for example:

```
rosrun openzen_sensor openzen_sensor_node _sensor_name:="LPMSCU2000573"
```

If your sensor is configured for a different baud rate, you can use the baudrate parameter to
give a specfic baud rate setting:

```
rosrun openzen_sensor openzen_sensor_node _sensor_name:="LPMSCU2000573" _baudrate:=115200
```

Now you can print the IMU values from ROS with:

```
rostopic echo /imu/data
```

Or plot some values (for example linear acceleration) with 

```
rosrun rqt_plot rqt_plot /imu/data/linear_acceleration
```

If you want to readout the values of two OpenZen sensors simultanously, you need to rename the topics and the node names likes this:

```
rosrun openzen_sensor openzen_sensor __name:="cu2node" _sensor_name:="LPMSCU2000573" imu:=/cu2_imu 
rosrun openzen_sensor openzen_sensor __name:="ig1_node" _sensor_name:="LPMSIG1000032" imu:=/ig1_imu
```

Alternatively, we have prepared a sample launch file openzen_lpms_ig1.launch to demonstrate data acquisition and plotting using openzen_sensor_node:
```
roslaunch openzen_sensor openzen_lpms_ig1.launch
```


## API
Please refer to our [ROS API documentation](https://lpresearch.bitbucket.io/openzen/latest/ros.html#ros-api) for further details. 
