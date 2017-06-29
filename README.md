px4_offboard_interface Exampe
========================

This is a simple MAVLink to interface example for Unix-like systems that can 
allow communication between Pixhawk and an offboard computer.

This example will recieve one MAVlink message and send one MAVlink message.


1. Building
========

```
$ cd px4_offboard_interface/
$ make
```

2. Run the Example Executable
-----------------------------

```
$ cd px4_offboard_interface/
$ ./px4_offboard_control  -d /dev/ttyUSB0 -b 57600
```

To stop the program, use the key sequence ```Ctrl-C```.


3. Here's an example output:

```
OPEN PORT
Connected to /dev/ttyUSB0 with 57600 baud, 8 data bits, no parity, 1 stop bit (8N1)

START READ THREAD 

CHECK FOR MESSAGES
Found

GOT VEHICLE SYSTEM ID: 1
GOT AUTOPILOT COMPONENT ID: 1

INITIAL POSITION XYZ = [ 0.0145 , 0.0254 , -0.0009 ] 
INITIAL POSITION YAW = 0.0016 

START WRITE THREAD 

IN ARMED MODE

ENABLE OFFBOARD MODE

SEND OFFBOARD COMMANDS
POSITION SETPOINT XYZ = [ 0.0145 , 0.0254 , -5.0009 ] 
POSITION SETPOINT YAW = 0.5016 
POSITION SETPOINT XYZ = [ 3.0145 , 4.0254 , -5.0009 ] 

0 CURRENT POSITION XYZ = [  0.0167 ,  0.0232 , -0.0002 ] 
1 CURRENT POSITION XYZ = [  0.0167 ,  0.0232 , -0.0002 ] 
2 CURRENT POSITION XYZ = [  0.0167 ,  0.0232 , -0.0002 ] 
3 CURRENT POSITION XYZ = [  0.2366 ,  0.4232 , -4.9290 ] 
4 CURRENT POSITION XYZ = [  2.5259 ,  3.4877 , -4.9987 ] 
5 CURRENT POSITION XYZ = [  3.2521 ,  4.3785 , -5.0051 ] 
6 CURRENT POSITION XYZ = [  3.3595 ,  4.4910 , -4.9979 ] 
7 CURRENT POSITION XYZ = [  2.8426 ,  3.8541 , -4.9823 ] 
8 CURRENT POSITION XYZ = [  2.7094 ,  3.6664 , -4.9793 ] 
9 CURRENT POSITION XYZ = [  2.9432 ,  3.9022 , -4.9902 ] 
...
100 CURRENT POSITION XYZ = [  2.9877 ,  4.0419 , -5.0041 ] 
101 CURRENT POSITION XYZ = [  3.0007 ,  4.0452 , -5.0043 ] 
102 CURRENT POSITION XYZ = [  3.0007 ,  4.0452 , -5.0043 ] 
103 CURRENT POSITION XYZ = [  3.0327 ,  4.0602 , -5.0009 ] 
104 CURRENT POSITION XYZ = [  3.0346 ,  4.0493 , -5.0008 ] 
105 CURRENT POSITION XYZ = [  3.0282 ,  4.0298 , -5.0028 ] 

DISABLE OFFBOARD MODE

DISARM MODE

READ SOME MESSAGES 
Got message LOCAL_POSITION_NED (spec: https://pixhawk.ethz.ch/mavlink/#LOCAL_POSITION_NED)
    pos  (NED):  3.024098 4.019777 -5.005819 (m)
Got message HIGHRES_IMU (spec: https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU)
    ap time:     195455939 
    acc  (NED):   0.014750 -0.040604 -9.708628 (m/s^2)
    gyro (NED):   0.007212  0.076203  0.002922 (rad/s)
    mag  (NED):   0.812265 -0.406478  1.780328 (Ga)
    baro:        0.000000 (mBar) 
    altitude:    493.015381 (m) 
    temperature: 0.000000 C 

CLOSE THREADS

CLOSE PORT

```

4. Exploration
===========

There are a few things to explore past this example.

For steps 2 and 3 from the above tutorial, you'll use a different port.  
On the off-board computer side, the port might now be ```/dev/ttyUSB0```.  

Now add a print statement in the Pixhawk Firmware to see received messages.  
Build and upload this to Pixhawk.

```
[Firmware/src/modules/mavlink/mavlink_receiver.cpp : line 1351]
/* if read failed, this loop won't execute */
for (ssize_t i = 0; i < nread; i++) {
	if (mavlink_parse_char(_mavlink->get_channel(), buf[i], &msg, &status)) {

		/* --- REPORT HANDLING OF MESSAGE --- */
		printf("\n");
		printf("HANDLE MESSAGE\n");
		printf("MSGID:%i\n",msg.msgid);

		/* handle generic messages and commands */
		handle_message(&msg);
```

On the off-board side, in another terminal run the ```px4_offboard_interface``` executable. 
You should see output in the NuttX shell similar to this:

```
HANDLE MESSAGE
MSGID:76

HANDLE MESSAGE
MSGID:84

(...)

HANDLE MESSAGE
MSGID:84

HANDLE MESSAGE
MSGID:76
```

Past this, you can:
- Modify the received message data type
- Modify the sent message data type