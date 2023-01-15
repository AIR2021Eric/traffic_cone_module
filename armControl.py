import cv2
import socket
import time
import math
# import rs485_test as rs485

# rs485.relay_control(rs485.allOFF)

HOST = '192.168.58.2'
PORT = 8080

controlType = {'MoveJ':                     201,
               'MoveL':                     203,
               'GetActualJointPosDegree':   377,
               'GetActualTCPPose':          377,
               'GetForwardKin':             377,
               'GetInverseKin':             377}

armSpeed = 50
armAcc = 50
armOvl = 80


def generateMessage(armFunction,functionInput):
    mode = controlType.get(armFunction)
    data = f'{armFunction}({functionInput})'
    # print(data)
    length = len(data)
    # print(len(data))
    return '/f/bIII52III{}III{}III{}III/b/f'.format(mode, length, data)


def TCP_Send_Receive(sendData):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    # print('connect')

    # print('send: ' + sendData)
    # print(sendData.encode())
    s.send(sendData.encode())

    indata = s.recv(1024)
    # print(f'indata: {indata}')
    if len(indata) == 0:  # connection closed
        s.close()
        # print('server closed connection.')

    # print('recv: ' + indata.decode())
    # recv: /f/bIII52III377III65III-133.881135,-81.602964,-101.270885,-30.337712,89.621244,94.063636III/b/f

    receive = indata.decode().split('III')
    # receive = receive.split('III')
    # print(receive[-2])
    # -133.881135,-81.602964,-101.270885,-30.337712,89.621244,94.063636
    return receive[-2]


def Get_Actual_Joint_Pos_Degree():
    GetActualJointPosDegree = generateMessage('GetActualJointPosDegree','')
    # print(GetActualJointPosDegree)
    ActualJointPosDegree = TCP_Send_Receive(GetActualJointPosDegree)
    # print(ActualJointPosDegree)
    # ActualJointPosDegree = ActualJointPosDegree.split(',')
    ActualJointPosDegree = list(map(float, ActualJointPosDegree.split(',')))
    # print(list(map(float, ActualJointPosDegree)))
    # print(ActualJointPosDegree[0])  # float
    return ActualJointPosDegree


def Get_Forward_Kin(jointPosDegree):
    GetForwardKin = generateMessage('GetForwardKin',jointPosDegree)   # get xyzRxRyRz
    # print(GetForwardKin)
    ForwardKin = TCP_Send_Receive(GetForwardKin)
    # print(ForwardKin)
    ForwardKin = list(map(float, ForwardKin.split(',')))
    return ForwardKin

def Get_Actual_TCP_Pose():
    GetActualTCPPose = generateMessage('GetActualTCPPose','')
    # print(GetActualTCPPose)
    ActualTCPPose = TCP_Send_Receive(GetActualTCPPose)
    # print(ActualTCPPose)
    ActualTCPPose = list(map(float, ActualTCPPose.split(',')))
    # print(list(map(float, ActualTCPPose)))
    # print(ActualTCPPose) #float
    return ActualTCPPose


def Get_Inverse_Kin(TCPPose):

    GetInverseKin = generateMessage('GetInverseKin', f"0,{TCPPose[0]},{TCPPose[1]},{TCPPose[2]},{TCPPose[3]},{TCPPose[4]},{TCPPose[5]},-1")    # get j1-j6
    # print(GetInverseKin)
    InverseKin = TCP_Send_Receive(GetInverseKin)
    # print(InverseKin)
    InverseKin = list(map(float, InverseKin.split(',')))
    return InverseKin


def Get_Target_TCP_Pos(currentJointPos, currentCoor, closest_cone_coor):
    angle = round(currentJointPos[0] * math.pi/180, 3)
    # print(f"angle: {angle}")
    # newX = xPan * math.cos(angle) - yPan * math.sin(angle)
    # newY = xPan * math.sin(angle) + yPan * math.cos(angle)

    #             arm       realsense
    # due left:   y (newX)  -x (xPan)
    # forward:    x (newY)  z (yPan)
    # upward:     z         -y
    # x,y,z = 0,0,-10

    # x, y, z = closest_cone_coor

    # closest_cone_coor = [x,y,z]  # realsense to arm
    # coordination_orientation = ForwardKin     # xyzRxRyRz
    coordination_orientation = currentCoor  # xyzRxRyRz
    x = closest_cone_coor[0] * math.cos(angle) - closest_cone_coor[1] * math.sin(angle)
    y = closest_cone_coor[0] * math.sin(angle) + closest_cone_coor[1] * math.cos(angle)
    z = closest_cone_coor[2]

    coordination_orientation[0] += x
    coordination_orientation[1] += y
    coordination_orientation[2] += z

    for i in range(len(coordination_orientation)):
        coordination_orientation[i] = round(coordination_orientation[i],3)

    return coordination_orientation


def Get_New_Joint_TCP_Pos(currentJointPos, currentCoor, closest_cone_coor):
    newCoor = Get_Target_TCP_Pos(currentJointPos, currentCoor, closest_cone_coor)
    newJointPos = Get_Inverse_Kin(newCoor)
    # print(f'newCoor     : {newCoor}')
    # print(f'newJointPos : {newJointPos}')
    return newJointPos, newCoor


def MoveJ(newJointPos, newCoor, armSpeed, armAcc, armOvl):
    j1, j2, j3, j4, j5, j6 = newJointPos
    x, y, z, Rx, Ry, Rz = newCoor
    MoveJ = generateMessage('MoveJ', f'{j1},{j2},{j3},{j4},{j5},{j6},{x},{y},{z},{Rx},{Ry},{Rz},0,0,{armSpeed},{armAcc},{armOvl},0.000,0.000,0.000,0.000,0,0,0,0,0,0,0,0')
    # MoveJ = generateMessage('MoveJ', f'{j1},{j2},{j3},{j4},{j5},{j6},{x},{y},{z},90,{Ry},{Rz},0,0,{armSpeed},{armAcc},{armOvl},0.000,0.000,0.000,0.000,0,0,0,0,0,0,0,0')
    print(MoveJ)
    TCP_Send_Receive(MoveJ)


def MoveL(newJointPos, newCoor, armSpeed, armAcc, armOvl):
    j1, j2, j3, j4, j5, j6 = newJointPos
    x, y, z, Rx, Ry, Rz = newCoor
    MoveL = generateMessage('MoveL', f'{j1},{j2},{j3},{j4},{j5},{j6},{x},{y},{z},{Rx},{Ry},{Rz},0,0,{armSpeed},{armAcc},{armOvl},0,0.000,0.000,0.000,0.000,0,0,0,0,0,0,0,0')
    # MoveL = generateMessage('MoveL', f'{j1},{j2},{j3},{j4},{j5},{j6},{x},{y},{z},90,{Ry},{Rz},0,0,{armSpeed},{armAcc},{armOvl},0,0.000,0.000,0.000,0.000,0,0,0,0,0,0,0,0')
    print(MoveL)
    TCP_Send_Receive(MoveL)

if __name__ == '__main__':
    # pos_standby_joint = [34.953275, -107.173698, 105.075635, 3.310257, 77.28871, 0.752509]
    # pos_standby_coor = [-268.24408, -453.874908, 754.079041, 91.18309, -1.01915, -42.353725]
    # #
    # pos_pick_joint = [32.030041, -42.427024, 114.638744, -61.54367, 82.973319, 1.35295]
    # pos_pick_coor = [-549.259399, -588.932739, -37.238522, 100.598709, -2.627484, -51.312431]
    # #
    # pos_pick_upper_joint = [32.030041, -52.472661, 115.113658, -51.973164, 82.973319, 1.352732]
    # pos_pick_upper_coor = [-549.252014, -588.922546, 83.18544, 100.598259, -2.627339, -51.312164]
    # #
    # pos_drop_joint = [72.216942, -49.165885, 115.203507, -55.369353, 82.973319, 1.35295]
    # pos_drop_coor = [-39.578671, -804.332886, 42.625816, 100.598488, -2.627071, -11.125664]
    #
    # pos_drop_upper_joint = [72.217377, -52.472444, 115.113876, -51.973164, 82.973319, 1.352515]
    # pos_drop_upper_coor = [-39.569981, -804.326477, 83.18248, 100.59848, -2.627111, -11.125014]
    #
    # MoveL(pos_standby_joint, pos_standby_coor, armSpeed, armAcc, armOvl)

    # get position
    currentJointPos = Get_Actual_Joint_Pos_Degree()
    currentCoor = Get_Actual_TCP_Pose()
    print(f"currentJointPos: {currentJointPos}")
    print(f"currentCoor:     {currentCoor}")