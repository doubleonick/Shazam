To ensure the ouster is working visit (Use the ouster ID from the ouster your are using):
http://os1-991947000828.local
or 
http://os1-992007000208.local
Depending on SN of the Ouster.

To find network settings visit:
http://os1-991947000828.local/api/v1/system/network

To examine other settings look at the web api in the ouster documentation.

===============================================================
Networking:
1. Do not use USB ethernet adapters for the Ouster, the ones we have do not support PTP for time synchronization.
2. Use a manual IP address setting (do not use DHCP) on the computer talking to the ouster and set the MAC address to only use the onboard NIC. The network config named OUSTER_OS1-64 works for this.
3. By default the Ouster has an ip address of: 169.254.156.128. If reset the ouster this is the IP address it has.
4. The instructions mention using a 10.5.5.0/24 subnet. This only works if you successfully follow their instructions to run a DHCP server on the computer. This is time consuming and not necessary. Just create a manual configuration in Network manager to connect to the ouster.
5. Netmask should be 255.255.0.0, leaving the last two octets free to be changed by the Ouster.
6. To avoid confusion, and force connections be directly through the PCI Ethernet port, set the MAC Address to the address of eno1.  This way, there is no confusion of what the correct setup is, and if anyone connects to a USB Ethernet adapter, the Ouster connection will not even show up.
===============================================================
ROS Code to use:

Do not use the code from apt-get, it is outdated and does not work well with firmware version 1.13 and later.

Use the code found at:
git clone https://github.com/ouster-lidar/ouster_example.git

clone this into your local catkin_ws and run catkin_make.

==============================================================
To setup the PTP server to synch the clocks between the ouster and computer follow the instructions in the ouster manual appendix part 1.4.
- Do not do section 1.4.7, this refers to synching to another master clock and we do not need or want that.


==============================================================
Here is the cli to run this:

roslaunch ouster_ros ouster.launch sensor_hostname:=os1-992007000208.local udp_dest:=169.254.156.200 viz:=false timestamp_mode:=TIME_FROM_PTP_1588

roslaunch ouster_ros ouster.launch sensor_hostname:=os1-991947000828.local udp_dest:=169.254.156.200 viz:=false timestamp_mode:=TIME_FROM_PTP_1588


The ouster IP address may change with out warining. So for hostname always use:os1-992007000208.local and not the IP address.

NOTE: It may take a few minutes for the ouster to initialize

==============================================================
To vizualize in rviz open a terminal and type rviz

Once rviz starts choose "Add"
Choose "By Topic"
Choose "Pointcloud2" as the topic.

This will often generate a frame error in rviz depending on your default rviz file.
To fix this change the reference frame:
Under Displays window, choose "Global Frames", "Fixed Frame" and scroll down until you find the ouster frame.

After this you should see a visualization of the point cloud.

