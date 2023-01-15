import pyrealsense2 as rs
import numpy as np
import cv2
import os
import yolov7_traffic_cone as yolo
import random
import onnxruntime as ort
import time
import armControl
import relayControl

# Setup inference for ONNX model
cuda = True
# w = "traffic_cone_yolov7.onnx"
w = "yolov7_416_best.onnx"
providers = ['CUDAExecutionProvider', 'CPUExecutionProvider'] if cuda else ['CPUExecutionProvider']
session = ort.InferenceSession(w, providers=providers)
names = ['blue cone', 'traffic cone', 'yellow cone']
colors = {name: [random.randint(0, 128) for _ in range(3)] for i, name in enumerate(names)}

# img = cv2.imread('../Resources/cones2.jpg')
# print(img.shape)

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipe_profile = pipeline.start(config)

# curr_frame = 0

# init x y position to collect depth
x=320
y=240

armSpeed = 50
armAcc = 50
armOvl = 80

t_old = 0
t_new = 0

storageJointPos = [72.216942, -49.165885, 115.203507, -55.369353, 82.973319, 1.35295]
storageCoor = [-39.578671, -804.332886, 42.625816, 100.598488, -2.627071, -11.125664]

upperStorageJointPos = [72.217377, -52.472444, 115.113876, -51.973164, 82.973319, 1.352515]
upperStorageCoor = [-39.569981, -804.326477, 83.18248, 100.59848, -2.627111, -11.125014]
try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            print("No depth frame or color frame")
            continue

        # Intrinsics & Extrinsics
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)

        # print(depth_intrin.ppx, depth_intrin.ppy)

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        # print(len(color_image))

        cone_pos = yolo.inference(color_image)
        # print(cone_pos)
        cone_coors = []
        closest_cone_coor = []

        if len(cone_pos):
            for i,(x,y) in enumerate(cone_pos):
                try:
                    # print(x,y)
                    depth = depth_frame.get_distance(x, y)
                    depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], depth)
                    # print(depth_point)
                    cone_coors.append([round(depth_point[2]*350/2,3), round(-depth_point[0]*500/2,3), round(-depth_point[1]*500/2,3)])
                    # text = "%.5lf, %.5lf, %.5lf\n" % (depth_point[0]*100, depth_point[1]*100, depth_point[2]*100)
                    # print(text)
                    # f.write(text)
                    # print("Finish writing the depth img")
                    cv2.circle(color_image,(x, y),5,(255,255,0),2)
                except:
                    pass
            # x front
            # y left
            # z up
            if len(cone_coors):
                closest_cone_coor = cone_coors[0]

            for i, (x, y, z) in enumerate(cone_coors):
                if closest_cone_coor[0] > x:
                    closest_cone_coor = cone_coors[i]

        print(f"closest_cone_coor: {closest_cone_coor}")

        # arm control
        if len(closest_cone_coor):
            if closest_cone_coor[0] > 300:
                # pass
                # get arm coor
                # calculate and get target position
                # pan arm

                currentJointPos = armControl.Get_Actual_Joint_Pos_Degree()
                currentCoor = armControl.Get_Actual_TCP_Pose()
                newJointPos, newCoor = armControl.Get_New_Joint_TCP_Pos(currentJointPos, currentCoor, [0,closest_cone_coor[1],closest_cone_coor[2]])  # [30,0,0] = [x,y,z] = coor from realsense
                armControl.MoveL(newJointPos, newCoor, armSpeed, armAcc, armOvl)
                print("moved")

            elif closest_cone_coor[0] <= 300 and closest_cone_coor[0] > 0:
                # pass
                # save arm location
                # collect cone
                #     move above cone
                #     move to cone
                # pick up and bring to storage zone
                #     gripper grip cone
                #     move upward
                #     move to storage zone
                # return to saved location
                #     gripper release cone

                position_offset = 0.1  # in mm
                cone_picked = False

                movement1 = False
                movement2 = False
                movement3 = False
                movement4 = False
                movement5 = False
                movement6 = False
                old_t = time.time()

                # save arm location
                # get current joint position and tcp coordinate
                currentJointPos = armControl.Get_Actual_Joint_Pos_Degree()
                currentCoor = armControl.Get_Actual_TCP_Pose()
                # save position as start point
                startJointPos = currentJointPos
                startCoor = currentCoor
                # arm MoveL above cone
                coneUpperJointPos, coneUpperCoor = armControl.Get_New_Joint_TCP_Pos(currentJointPos, currentCoor,
                                                                        [-closest_cone_coor[0], closest_cone_coor[1],
                                                                         closest_cone_coor[2] + 500])  # [30,0,0] = [x,y,z] = coor from realsense
                armControl.MoveL(coneUpperJointPos, coneUpperCoor, armSpeed, armAcc, armOvl)
                while True:
                    new_t = time.time()

                    if new_t - old_t >= 0.5:
                        print("start iter")
                        # get current position
                        currentJointPos = armControl.Get_Actual_Joint_Pos_Degree()
                        currentCoor = armControl.Get_Actual_TCP_Pose()
                        print("collected current location")
                        # if distance between current location and target location is less than 0.5mm, prompt arm to action

                        # move to cone position
                        if (((currentCoor[0] - coneUpperCoor[0]) ** 2 + (
                                currentCoor[1] - coneUpperCoor[1]) ** 2 + (
                                     currentCoor[2] - coneUpperCoor[
                                 2]) ** 2) ** 0.5) < position_offset and not cone_picked:
                            coneJointPos, coneCoor = armControl.Get_New_Joint_TCP_Pos(currentJointPos,
                                                                                                currentCoor,
                                                                                                [0,0,-345])
                            armControl.MoveL(coneJointPos, coneCoor, armSpeed, armAcc, armOvl)
                            print("move to cone position")
                            movement1 = True

                        # pick up cone
                        elif movement1 and not cone_picked and (((currentCoor[0] - coneCoor[0]) ** 2 + (currentCoor[1] - coneCoor[1]) ** 2 + (
                                currentCoor[2] - coneCoor[2]) ** 2) ** 0.5) < position_offset :
                            # gripper grip cone
                            relayControl.send_message("relay2ON")
                            cone_picked = True
                            time.sleep(4)

                            armControl.MoveL(coneUpperJointPos, coneUpperCoor, armSpeed, armAcc, armOvl)
                            print("pick up cone")
                            movement2 = True

                        # move above storage zone
                        elif movement2 and (((currentCoor[0] - coneUpperCoor[0]) ** 2 + (
                                currentCoor[1] - coneUpperCoor[1]) ** 2 + (
                                     currentCoor[2] - coneUpperCoor[2]) ** 2) ** 0.5) < position_offset and cone_picked:
                            pos_drop_upper_joint = [72.217377, -52.472444, 115.113876, -51.973164, 82.973319, 1.352515]
                            pos_drop_upper_coor = [-39.569981, -804.326477, 83.18248, 100.59848, -2.627111, -11.125014]
                            armControl.MoveL(upperStorageJointPos, upperStorageCoor, armSpeed, armAcc, armOvl)
                            print("move above storage zone")
                            movement3 = True

                        # move to storage zone
                        elif movement3 and (((currentCoor[0] - upperStorageCoor[0]) ** 2 + (
                                currentCoor[1] - upperStorageCoor[1]) ** 2 + (
                                     currentCoor[2] - upperStorageCoor[
                                 2]) ** 2) ** 0.5) < position_offset and cone_picked:
                            storageJointPos = [72.216942, -49.165885, 115.203507, -55.369353, 82.973319, 1.35295]
                            storageCoor = [-39.578671, -804.332886, 42.625816, 100.598488, -2.627071, -11.125664]
                            armControl.MoveL(storageJointPos, storageCoor, armSpeed, armAcc, armOvl)
                            print("move to storage zone")
                            movement4 = True

                        # release cone and move above storage zone
                        elif movement4 and (((currentCoor[0] - storageCoor[0]) ** 2 + (currentCoor[1] - storageCoor[1]) ** 2 + (
                                currentCoor[2] - storageCoor[2]) ** 2) ** 0.5) < position_offset and cone_picked:
                            # release grapper
                            relayControl.send_message("relay2OFF")
                            cone_picked = False
                            time.sleep(1)
                            upperStorageJointPos = [72.217377, -52.472444, 115.113876, -51.973164, 82.973319, 1.352515]
                            upperStorageCoor = [-39.569981, -804.326477, 83.18248, 100.59848, -2.627111, -11.125014]
                            armControl.MoveL(upperStorageJointPos, upperStorageCoor, armSpeed, armAcc, armOvl)
                            print("release cone and move above storage zone")
                            movement5 = True

                        # move to pos_standby
                        elif movement5 and (((currentCoor[0] - upperStorageCoor[0]) ** 2 + (
                                currentCoor[1] - upperStorageCoor[1]) ** 2 + (
                                     currentCoor[2] - upperStorageCoor[2]) ** 2) ** 0.5) < position_offset and not cone_picked:
                            armControl.MoveL(startJointPos, startCoor, armSpeed, armAcc, armOvl)
                            print("move to pos_standby")
                            movement6 = True
                            break

                        elif movement6:
                            movement1 = False
                            movement2 = False
                            movement3 = False
                            movement4 = False
                            movement5 = False
                            movement6 = False
                        # else:
                        #     print("cannot reach destination")
                        #     break

                        old_t = new_t
                        print("end iter")

                print("finish")

        # cv2.rectangle(color_image, (x, y), (x+w, y+h), (255, 0, 0), 2)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))

        t_new = time.time()
        fps = 1 / (t_new - t_old)
        t_old = t_new
        cv2.putText(images, 'FPS = ' + str(round(fps, 3)), (10, 50), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 0, 255), 2)
        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

        # curr_frame += 1
finally:
    # Stop streaming
    pipeline.stop()