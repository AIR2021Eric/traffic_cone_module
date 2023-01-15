import socket
import time
import serial
import armControl
import relayControl

# ser = serial.Serial('COM4',baudrate=9600,timeout=1)
# time.sleep(0.5)
# print("relay connected")

HOST = '192.168.58.2'
PORT = 8080
# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.connect((HOST, PORT))
print('arm connected')

armSpeed = 50
armAcc = 50
armOvl = 80

position_offset = 0.1  # in mm

# pos_location_joint = [J1,J2,J3,J4,J5,J6]
# pos_location_coor = [X,Y,Z,RX,RY,RZ]

pos_standby_joint = [34.953275, -107.173698, 105.075635, 3.310257, 77.28871, 0.752509]
pos_standby_coor = [-268.24408, -453.874908, 754.079041, 91.18309, -1.01915, -42.353725]
#
pos_pick_joint = [32.030041, -42.427024, 114.638744, -61.54367, 82.973319, 1.35295]
pos_pick_coor = [-549.259399, -588.932739, -37.238522, 100.598709, -2.627484, -51.312431]
#
pos_pick_upper_joint = [32.030041, -52.472661, 115.113658, -51.973164, 82.973319, 1.352732]
pos_pick_upper_coor = [-549.252014, -588.922546, 83.18544, 100.598259, -2.627339, -51.312164]
#
pos_drop_joint = [72.216942, -49.165885, 115.203507, -55.369353, 82.973319, 1.35295]
pos_drop_coor = [-39.578671, -804.332886, 42.625816, 100.598488, -2.627071, -11.125664]

pos_drop_upper_joint = [72.217377, -52.472444, 115.113876, -51.973164, 82.973319, 1.352515]
pos_drop_upper_coor = [-39.569981, -804.326477, 83.18248, 100.59848, -2.627111, -11.125014]


# move to pos_standby
armControl.MoveL(pos_standby_joint,pos_standby_coor,armSpeed,armAcc,armOvl)
print("move to stand by")

cone_picked = False
old_t = time.time()
while True:
    new_t = time.time()

    if new_t-old_t >= 0.5:
        print("start iter")
        # get current position
        currentJointPos = armControl.Get_Actual_Joint_Pos_Degree()
        currentCoor = armControl.Get_Actual_TCP_Pose()
        print("collected current location")
        # if distance between current location and target location is less than 0.5mm, prompt arm to action

        # move to pos_pick_upper
        if (((currentCoor[0] - pos_standby_coor[0])**2 + (currentCoor[1] - pos_standby_coor[1])**2 + (
                currentCoor[2] - pos_standby_coor[2])**2)**0.5) < position_offset and not cone_picked:
            armControl.MoveL(pos_pick_upper_joint,pos_pick_upper_coor,armSpeed,armAcc,armOvl)
            print("move to pos_pick_upper")

        # move to pos_pick
        if (((currentCoor[0] - pos_pick_upper_coor[0]) ** 2 + (currentCoor[1] - pos_pick_upper_coor[1]) ** 2 + (
                currentCoor[2] - pos_pick_upper_coor[2]) ** 2) ** 0.5) < position_offset and not cone_picked:
            armControl.MoveL(pos_pick_joint,pos_pick_coor,armSpeed,armAcc,armOvl)
            print("move to pos_pick")

        # pick up cone and move to pos_pick_upper
        if (((currentCoor[0] - pos_pick_coor[0]) ** 2 + (currentCoor[1] - pos_pick_coor[1]) ** 2 + (
                currentCoor[2] - pos_pick_coor[2]) ** 2) ** 0.5) < position_offset and not cone_picked:
            relayControl.send_message("relay2ON")
            cone_picked = True
            time.sleep(4)

            armControl.MoveL(pos_pick_upper_joint,pos_pick_upper_coor,armSpeed,armAcc,armOvl)
            print("pick up cone move to pos_pick_upper")

        # move to pos_drop_upper
        if (((currentCoor[0] - pos_pick_upper_coor[0]) ** 2 + (currentCoor[1] - pos_pick_upper_coor[1]) ** 2 + (
                currentCoor[2] - pos_pick_upper_coor[2]) ** 2) ** 0.5) < position_offset and cone_picked:
            armControl.MoveL(pos_drop_upper_joint,pos_drop_upper_coor,armSpeed,armAcc,armOvl)
            print("move to pos_drop_upper")

        # move to pos_drop
        if (((currentCoor[0] - pos_drop_upper_coor[0]) ** 2 + (currentCoor[1] - pos_drop_upper_coor[1]) ** 2 + (
                currentCoor[2] - pos_drop_upper_coor[2]) ** 2) ** 0.5) < position_offset and cone_picked:
            armControl.MoveL(pos_drop_joint,pos_drop_coor,armSpeed,armAcc,armOvl)
            print("move to pos_drop")

        # release cone and move to pos_drop_upper
        if (((currentCoor[0] - pos_drop_coor[0]) ** 2 + (currentCoor[1] - pos_drop_coor[1]) ** 2 + (
                currentCoor[2] - pos_drop_coor[2]) ** 2) ** 0.5) < position_offset and cone_picked:
            relayControl.send_message("relay2OFF")
            cone_picked = False
            time.sleep(1)
            armControl.MoveL(pos_drop_upper_joint,pos_drop_upper_coor,armSpeed,armAcc,armOvl)
            print("release cone move to pos_drop_upper")

        # move to pos_standby
        if (((currentCoor[0] - pos_drop_upper_coor[0]) ** 2 + (currentCoor[1] - pos_drop_upper_coor[1]) ** 2 + (
                currentCoor[2] - pos_drop_upper_coor[2]) ** 2) ** 0.5) < position_offset and not cone_picked:
            armControl.MoveL(pos_standby_joint,pos_standby_coor,armSpeed,armAcc,armOvl)
            print("move to pos_standby")
            break

        old_t = new_t
        print("end iter")

print("finish")