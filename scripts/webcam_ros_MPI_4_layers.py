import sys
import math
from matplotlib.pyplot import winter
sys.path.append('/home/uav/openpose/build/python')
from openpose import pyopenpose as op
import cv2
import time
import joblib
import numpy as np
from collections import deque
from PIL import Image,ImageFont,ImageDraw
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from Wave2Reach.msg import ImageMsg


class OpenPose:
    def __init__(self): 
        self.image_sub = rospy.Subscriber('/image_data', ImageMsg, self.callback)
        self.depth_sub = rospy.Subscriber('/sensors/stereo_cam/depth/depth_registered', Image, self.callback1)
        self.pose_pub = rospy.Publisher('/ai_robot/findpath/targetP', Pose, queue_size=1)
        self.K = np.array([[525.9661254882812, 0.0, 629.2212524414062], [0.0, 525.9661254882812, 354.1390686035156], [0.0, 0.0, 1.0]])
        self.detected_flag = False
        self.point = (-1,-1)# center of human
        self.box = (-1,-1,-1,-1) # left, top, right, bottom
        self.target = (-1,-1)
        self.cntdown = 0
        self.newimgf = False
        self.newdepf = False


    def callback1(self, data):
        # depth_array = np.array(data, dtype=np.float32)
        depth_array = np.ndarray(shape=(data.height, data.width), dtype=np.float32, buffer=data.data)# 720 * 1280
        self.depth = depth_array 
        self.newdepf = True
        # v = np.int(data.width/2)#1280/2
        # u = np.int(data.height/2)#720/2
        


    def callback(self, data):
        global opWrapper

        ros_image = np.ndarray(shape=(data.height, data.width, data.channels), dtype=np.uint8, buffer=data.data) # 将自定义图像消息转化为图像
        # print("1111111111111111111")

        # image = cv2.resize(ros_image, dsize=(800,600))
        # print(ros_image.shape)
        self.image = ros_image
        self.newimgf=True


    def process(self):
        # rospy.loginfo('111 {}'.format(self.cntdown))
        if(self.cntdown > 0):
            self.cntdown -= 1
            return
        if(not self.newimgf):
            return
        if(not self.newdepf):
            return
        # rospy.loginfo('aaa')
        # Process Image
        image = self.image
        datum = op.Datum()
        datum.cvInputData = image
        opWrapper.emplaceAndPop(op.VectorDatum([datum]))
        output = datum.cvOutputData

        if datum.poseKeypoints is not None:
            human = datum.poseKeypoints[0]

            lx = (human[7,1] - human[1,1]) * 100 / (human[14,1] - human[1,1])
            rx = (human[4,1] - human[1,1]) * 100 / (human[14,1] - human[1,1])
            # print("lx=" + str(lx))
            # print("rx=" + str(rx))
            left, right = False, False

            if (human[7]!=0).any() and (human[0]!=0).any():
                if lx < -100:
                    left = True
            if (human[4]!=0).any() and (human[0]!=0).any():
                if rx < -100:
                    right = True
            if left and right:
                flag = "both hands"
            elif left:
                flag = "left hand"
            elif right:
                flag = "right hand"
            else:
                flag = "no hand"

            print(flag)
            cv2.putText(output, flag, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.putText(image, flag, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            # self.point = (np.int((human[8,0]+human[11,0])/2), np.int((human[1,1]+human[8,1])/2))
            self.point = (np.int(human[14,0]), np.int(human[14,1]))
            self.box = (np.int(human[8,0]), np.int((human[1,1]+human[14,1])/2), np.int(human[11,0]), np.int(human[11,1]))
            print(self.point)
            print(self.box)
            # print(human)
            cv2.circle(output, self.point, 5, (0,0,0), -1)
            cv2.rectangle(output, (self.box[0],self.box[1]), (self.box[2],self.box[3]), (0,0,255), 2)
            # id = 14
            # cv2.circle(output, (human[id,0], human[id,1]), 5, (0,0,0), -1)
            if flag in ("both hands", "left hand", "right hand"):
                self.detected_flag = True 

        cv2.imshow("OpenPose", output)
        # cv2.imwrite("ros_image.png", ros_image)
        cv2.waitKey(1)

        u = np.int(self.point[0])
        v = np.int(self.point[1])
        num = 0
        dist = 0
        
        if self.detected_flag == True:

            
            depth_array = self.depth
            for x in range(self.box[0], self.box[2]):
                for y in range(self.box[1], self.box[3]):
                    if not(math.isnan(depth_array[y, x])) and not(math.isinf(depth_array[y, x])):
                        dist += depth_array[y, x]
                        num +=1
            
            # print(num)
            if num != 0 and dist != 0:
                dist = dist / num
                print('Center aveage depth of human: {dist} m'.format(dist=dist))
                p = np.array([[self.point[0]], [self.point[1]], [1]])
                p = np.mat(p)
                Kalib = np.mat(self.K)
                P = dist * Kalib.I * p
                print(P)
                self.target = (P[0], P[2]-0.8)
                pose = Pose()
                pose.position.x = self.target[1]
                pose.position.y = -self.target[0]
                pose.position.z = 0
                pose.orientation.x = 0
                pose.orientation.y = 0
                pose.orientation.z = 0
                pose.orientation.w = 1
                self.pose_pub.publish(pose)
                print("pub depth value of the person")

                # self.point = (0,0)
                # self.box = (0,0,0,0)
                self.cntdown = 50
                self.newimgf = False
                self.newdepf = False
                self.detected_flag = False

if __name__ == '__main__':

    params = dict()
    params["model_folder"] = "/home/uav/openpose/models/"
    # BODY_25, COCO, MPI, MPI_4_layers
    params["model_pose"] = "MPI_4_layers"

    # Starting OpenPose
    opWrapper = op.WrapperPython()
    opWrapper.configure(params)
    opWrapper.start()

    OP = OpenPose()
    rospy.init_node('OpenPose', anonymous=True)

    # cap = cv2.VideoCapture(0)
    cnt = 0
    tmp_flag = "no hand"
    last_flag = "no hand"
    rate = rospy.Rate(5)

    try:
        while not rospy.is_shutdown():
            OP.process()
            rate.sleep()
    except KeyboardInterrupt:
        print('Shutting down')
    cv2.destroyAllWindows()

