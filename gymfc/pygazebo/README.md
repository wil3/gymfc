Pygazebo is a Python library originally developed by Josh Pieper enabling communication with Gazebo through the
Google Probobuf API. The original project can be found here, https://github.com/jpieper/pygazebo.

This project appears to be abandoned as of 2016. Until [bugs](
#https://bitbucket.org/osrf/gazebo/issues/2397/gzserver-doesnt-close-disconnected-sockets
) are resolved in the gz tool, this modified version included in the GymFC
repository is the main method for GymFC to communicate
with Gazebo. 

Changes, 
* Support for Python 3.5+
* Support for Gazebo 8

Note, only Protobuf messages that are required by GymFC are included.
