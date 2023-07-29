from pymavlink import mavutil
from time import sleep

wp = [(35.154601, 128.104636),(35.155009, 128.104528),(35.154655, 128.104324),(35.154883, 128.104708),(35.154880, 128.104277)]

wp1 = [(35.1513876, 128.1007150), (35.15151443, 128.100666)]
wp2 = [(35.1547339 , 128.1046750), (35.1548539 , 128.1041677)]

def recvmsg(master, types):
    msg = master.recv_match(type=types, blocking=True)
    print(msg)
    return msg

def connect():
	master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
	master.wait_heartbeat()
	print('Drone is connected!')
	return master

def missionstart(master):
	master.mav.command_long_send(master.target_system, master.target_component,
	mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)
	recvmsg(master, 'COMMAND_ACK')


def missionupload(master, latitude, longitude):
	master.mav.mission_count_send(master.target_system, master.target_component, 1)
	msg = master.mav.mission_item_encode(
		master.target_system, master.target_component,
		0,
		mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
		0, 1, 0, 0, 0, 0,
		latitude, longitude, 5)
	master.mav.send(msg)

	while True:
		recv_msg = master.recv_match(type=['MISSION_ACK', 'MISSION_REQUEST'], blocking=True)
		if recv_msg.get_type() == 'MISSION_ACK':
			if (recv_msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED):
				print("미션 아이템 전송 성공")
			else:
				print("미션 아이템 전송 실패")
			break
		elif recv_msg.get_type() == 'MISSION_REQUEST':
			continue

def missionupload1(master):
	master.mav.mission_count_send(master.target_system, master.target_component,2)
	for i in range(2):
		msg = master.mav.mission_item_encode(
		master.target_system, master.target_component,i,
		mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
		0, 1, 0, 0, 0, 0,
		wp2[i][0], wp2[i][1], 5)
		master.mav.send(msg)

	while True:
		recv_msg = master.recv_match(type=['MISSION_ACK', 'MISSION_REQUEST'], blocking=True)
		if recv_msg.get_type() == 'MISSION_ACK':
			if (recv_msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED):
				print("미션 아이템 전송 성공")
			else:
				print("미션 아이템 전송 실패")
			break
		elif recv_msg.get_type() == 'MISSION_REQUEST':
			continue

def missiondownload(master):
	master.mav.mission_request_list_send(master.target_system, master.target_component)

	msg = master.recv_match(type='MISSION_COUNT', blocking=True)
	if msg:
		num_missions = msg.count
		print("다운로드할 미션 수:", num_missions)

	if num_missions > 0:
		for mission_seq in range(num_missions):
			master.mav.mission_request_int_send(master.target_system, master.target_component, mission_seq)

		while True:
			msg = master.recv_match(type=['MISSION_ITEM', 'MISSION_ITEM_INT'], blocking=True)
			if msg.get_type() == 'MISSION_ITEM' or msg.get_type() == 'MISSION_ITEM_INT':
				print("미션 아이템:", msg)
				break

def sendwaypoint(master, latitude, longitude):
	master.mav.mission_item_send(master.target_system, master.target_component, 0, 3, 
	mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 
	3, 1, 0, 0, 0, 0, 
	latitude, longitude, 4)
	recvmsg(master, 'MISSION_ACK')

def setautomode(master):
	master.mav.command_long_send(master.target_system, master.target_component,
	mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
	0, 157, 4, 4, 0, 0, 0, 0)
	msg = recvmsg(master, 'COMMAND_ACK')
	msg = msg.to_dict()
	print(mavutil.mavlink.enums['MAV_RESULT'][msg['result']].description)

def setautomode1(master):
	master.mav.command_long_send(master.target_system, master.target_component,
	mavutil.mavlink.MAV_CMD_DO_SET_MODE,
	0, 4, 0, 0, 0, 0, 0, 0)
	msg = recvmsg(master, 'COMMAND_ACK')
	msg = msg.to_dict()
	print(mavutil.mavlink.enums['MAV_RESULT'][msg['result']].description)

def setstabmode(master):
	master.mav.command_long_send(master.target_system, master.target_component,
	mavutil.mavlink.MAV_CMD_DO_SET_MODE,
	0, 209, 7, 0, 0, 0, 0, 0)
	msg = recvmsg(master, 'COMMAND_ACK')
	msg = msg.to_dict()
	print(mavutil.mavlink.enums['MAV_RESULT'][msg['result']].description)

def clearallmission(master):
	master.mav.mission_clear_all_send(master.target_system, master.target_component)
	recvmsg(master, 'MISSION_ACK')

def armdisarm(master, i):
	master.mav.command_long_send(master.target_system, master.target_component,
	mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
	0, i, 0, 0, 0, 0, 0, 0)
	msg = master.recv_match(type='COMMAND_ACK', blocking=True)
	msg = msg.to_dict()
	print(msg)
	print(mavutil.mavlink.enums['MAV_RESULT'][msg['result']].description)

def land(master):
	speed = 0;
	master.mav.command_long_send(master.target_system, master.target_component,
	mavutil.mavlink.MAV_CMD_NAV_LAND, 
	0, speed, 0, 0, 0, 0, 0, 0)
	recvmsg(master, 'COMMAND_ACK')

def force_shutdown(master):
	master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
	0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
	0,0,0,0,0)

def setmissioncurrent(master, seq):
	master.mav.command_long_send(master.target_system, master.target_component,
	mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
	0, 0, seq, 0, 0, 0, 0, 0)
	recvmsg(master, "COMMAND_ACK")

if __name__ == '__main__':
	lat = 35.154779
	long = 128.104114
	lat1 = 35.154590
	long1 = 128.092980
	master = connect()
	print('1. download mission\n2. upload mission\n3. upload mission 1')
	print('4. set automode\n5. clear all mission\n6. ARM\n7. Disarm')
	print('8. land\n9. shutdown\n10. set stabilize mode')
	print('11. start mission mode\n 12. set location')
	print('13. print latitude\n 14.set mission current 16.set mission current 1')
	print('0. exit')
	while(1):
		i = int(input('input number!'))
		if(i == 1):
			missiondownload(master)
		elif(i == 2):
			missionupload(master, lat, long)
		elif(i == 3):
			missionupload1(master)
		elif(i == 4):
			setautomode(master)
		elif(i == 5):
			clearallmission(master)
		elif(i == 6):
			armdisarm(master, 1)
		elif(i == 7):
			armdisarm(master, 0)
		elif(i == 8):
			land(master)
		elif(i == 9):
			force_shutdown(master)
		elif(i == 10):
			setstabmode(master)
		elif(i == 11):
			missionstart(master)
		elif(i == 12):
			lat = double(input('Set latitude!'))
			long = double(input('Set longitude!'))
		elif(i == 13):
			print(lat, long)
		elif(i == 14):
			setmissioncurrent(master, 0)
		elif(i == 15):
			setautomode1(master)
		elif(i == 16):
			setmissioncurrent(master, 1)
		elif(i == 0):
			master.close()
			break;
