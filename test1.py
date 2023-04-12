from pymavlink import mavutil
import keyboard
import time

val = [
    # x, y, z, r
    [0, 0, 0, 0], #land 0
    [250, 0, 500, 0], #front 1
    [-250, 0, 500, 0], #back 2
    [0, 250, 500, 0], #right 3
    [0, -250, 500, 0], #left 4
    [0, 0, 1000, 0], # up 5
    [0, 0, 250, 0], # down 6
    [0, 0, 500, 0] # hold 7
];

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()

print('Drone is connected!')

armed = False

while 1:
	i = 7;
	if keyboard.is_pressed('d'):
		print('Start!')
		master.mav.command_long_send(master.target_system, master.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
		print("Waiting for the vehicle to arm")
		master.motors_armed_wait()
		print('Armed!')
		armed = True;
	if keyboard.is_pressed('f'):
		print('stop!')
		master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
		master.motors_disarmed_wait()
		armed = False;
	if keyboard.is_pressed('left'):
		print('Press Left!')
		i = 4
	if keyboard.is_pressed('right'):
		print('Press Right!')
		i = 3
	if keyboard.is_pressed('up'):
		print('Press front!')
		i = 1
	if keyboard.is_pressed('down'):
		print('Press back!')
		i = 2
	if keyboard.is_pressed('space'):
		print('Press space bar!')
		i = 5
	if keyboard.is_pressed('ctrl'):
		print('Press Control!')
		i = 6
	if keyboard.is_pressed('z'):
		print('Press z!')
		i = 7
	if keyboard.is_pressed('l'):
		print('Press l!')
		i = 0
	if(armed):
		master.mav.manual_control_send(master.target_system, val[i][0], val[i][1], val[i][2], val[i][3], 0)
	time.sleep(0.085)
