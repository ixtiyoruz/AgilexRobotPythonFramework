# AgilexRobotPythonFramework
python-can wrapped framework to parse agilex can-bus data .
 |no |  tested on |  
 |:-:| :-------------------: | 
 | 1 | scout mini |
 
requirements:
wake up the can interface:
<pre>
sudo ip link set can0 down type can bitrate 500000
</pre>

<pre>
  python3, ubuntu
  sudo apt-get install net-tools
  pip3 install python-can
  pip3 install simple-pid
  pip3 install matplotlib
  pip3 install numpy
</pre>

reference repos:

 |no |  name |  
 |:-:| :-------------------: | 
 | 1 | https://github.com/ninedraft/python-udp |
 | 2 | https://github.com/hardbyte/python-can | 
 | 3 | https://github.com/agilexrobotics/agx_sdk | 
 
 Issues:
 1. if you face the problem below, then try to use different usb port it worked for me.
<pre>
Failed to transmit: [Errno 105] No buffer space available
</pre>
