# Jackal User Setup Guide

### Goal

This documentation is meant to help an end user to bring a Clearpath Jackal to the point of being able to perform autonomous reconnaissance missions.  
The user will be able to perform keyboard or game controller teleoperation for missions, or for troubleshooting.  The user will also be able to use image_view 
or rviz to visualize LiDAR and depth camera images.  [MORE?]

### Prerequisites

The capabilities discussed below can be examined as modules associated with the Jackal platform, or individual sensors, or as more complex behaviors integrating 
one or more sensors with the mobile platform.  The modules can be seen as building blocks and or troubleshooting tools for developing the bigger behaviors.  
If approached in a certain order, the capabilities will build on each other.  The prerequisites are as follows based on context.

  | Context                                         | Jackal      | Intel RS D435i    | GC Computer | Monitor | Mouse  | Keyboard | Joystick |
  | :---                                            |    :----:   |    :----:         |    :----:   | :----:  | :----: | :----:   |     ---: |
  | Teleoperation                                   | Y           | N                 | O           | Y       | Y      | O        | O        |
  | Intel RS D435i Image Viz                        | N           | Y                 | Y           | N       | N      | N        | N        |
  | Intel RS D435i Pointcloud Viz                   | N           | Y                 | Y           | O       | O      | O        | N        |
  | Intel RS D435i PC to LaserScan                  | N           | Y                 | Y           | O       | O      | O        | N        |
  | Intel RS D435i SLAM                             | N           | Y                 | Y           | O       | O      | O        | N        |
  | Google Cartographer w/Jackal and Intel RS D435i | Y           | Y                 | Y           | O       | O      | O        | O        |

### Network Configuration

Network configuration in this context refers to setting up the hardware, and software settings to enable the Jackal onboard computer, companion computer, 
and ground station computer to all be on the same network, and communicate smoothly among machines using ROS.  The strategy entails configuring the companion 
computer to be a mobile WiFi hospot to which the onboard and ground control computers will both connect.

- On your companion computer (e.g. NUC, Brix, etc.), open the network manager connection editor:

`nm-connection-editor`
- Choose the plus sign to add a connections
- Choose Wi-Fi for "Connection Type" and click "Create..."
- On the Wi-Fi tab:
  - Set "Connection Name:" to [Companion Name]Hotspot (e.g. NUC40Hotspot]
  - Set "Mode:" to Hotspot
  - Set "SSID:" to [Companion Name] (e.g. NUC40)
- On the Wi-Fi Security Tab
  - Set "Security:" to WPA & WPA2 Personal
  - Set the password: nuc40access (or similar based on the name of your companion machine)
- On the General Tab
  - Make sure 'Automatically connect ...' is checked

### Editing .bashrc

You will want to edit the root .bashrc file on your development/ground control machine, and your companion computer.  Do not edit files on the onboard Jackal
computer.  If you suspect you may need to edit a file on the onboard computer, please consult your instructor or an advisor.  And remember, for all examples
below, your IP addresses may be different than what is shown.

In a terminal, in the home directory ("~"), use your favorite editor to edit .bashrc:
- `sudo gedit .bashrc`

The minimum to do here is to export the ROS Master and ROS IP values:
- `ifconfig`
- Look for the wireless IP address: this will be your ROS_IP
- Open your WiFi Settings and find the network you set up in the "Network Configuration" section.  The IP address of this network will be your ROS_MASTER_URI
- `export ROS_MASTER_URI=http://10.42.0.127:11311`
- `epxort ROS_IP=10.212.199.192`

You may also want to create an alias for connecting to the Jackal companion computer if the .bashrc file you are editing is on your dev computer:
- `alias ssh-jackal-companion='ssh user1@10.212.199.192`

Once you are done editing, save your work and exit your text editor.  Now it's time to source .bashrc within your work environment:
- `source .bashrc`

**NOTE:** For development, you may want a secondary bashrc file, e.g. ".dev-bashrc".  You will want to modify this in a similar fashion, but with ROS_MASTER_URI
being the same as ROS_IP.  Which is to say, you are setting your development machine to be ROS Master.  This makes it easy to plug a sensor into your development 
machine and experiment with different packages and nodes without having to worry about your entire robot setup, including power, etc.

### Generic Teleoperation

[ros-teleop](https://github.com/ros-teleop/teleop_twist_keyboard), [jackal](https://github.com/jackal)

### Intel RealSense

[librealsense (drivers)](https://github.com/IntelRealSense/librealsense), [realsense-ros](https://github.com/IntelRealSense/realsense-ros), [SLAM](https://github.com/IntelRealSense/realsense-ros/wiki/SLAM-with-D435i), [rs_slam](https://intel.github.io/robot_devkit_doc/pages/rs_slam.html), [jackal-cartographer-navigation](https://github.com/jackal/jackal_cartographer_navigation), [pointcloud to laserscan wiki](http://wiki.ros.org/pointcloud_to_laserscan), [pointcloud to laserscan repo](https://github.com/ros-perception/pointcloud_to_laserscan/tree/foxy/launch)
