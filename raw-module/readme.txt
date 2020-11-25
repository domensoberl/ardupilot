This module adds the capability to control quadcopter motors individually from Python. The name of the module is RAW.

Make sure the git checkout is recursive:
git clone --recursive https://github.com/domensoberl/ardupilot.git

Installation
------------
Install our adaptation of the Mavlink protocol.
1. Copy (overwrite) the new Mavlink message definitions ardupilotmega.xml to
   ardupilot/modules/mavlink/message_definitions/v1.0/
2. Uninstall the currently installed MavLink package:
   pip uninstall pymavlink
3. Go to ardupilot/modules/mavlink/pymavlink and install our custom version:
   python setup.py install --user
4. Copy the module mavproxy_raw.py to
   ~/.local/lib/python3.x/site-packages/MAVProxy/modules/
   where python3.x is the version of your Python installation, e.g. python3.8

Running the demo
----------------
1. Run the simulator:
   gazebo worlds/iris_arducopter_demo.world
2. Run the SITL platform. In the ardupilot/ArduCopter directory run:
   sim_vehicle.py -f gazebo-iris
3. In the SITL console, load the module RAW:
   > module load raw
   This module starts a TCP/IP server on port 6200. Only one client is allowed to connect. When connected, it starts receiving pitch, roll, yaw, altitude values continuously.
4. Switch the quadcopter to the RAW mode (mode 28) by typing:
   > mode 28
   In this mode, the low level motor outputs are enabled. All security checks are circumvented. If switched to some other mode while in flight, rotors will most likely stop and the copter will crash.
5. Run the demo.py program. Quadcopter can now be controlled through GUI.