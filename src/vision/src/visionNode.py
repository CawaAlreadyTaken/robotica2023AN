import rospy
import numpy as np
from math import sin, cos
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision.msg import custMsg

class VisionNode():
    def __init__(self):
        self.model = YOLO("weights.pt")
        self.TAVOLO_HEIGHT = 0.87
        self.width = 1920
        self.height = 1080
        rospy.init_node('neuralNetwork')
        rospy.Subscriber("/ur5/zed_node/left/image_rect_color",
                         Image, callback= self.getAndSend3dCoords, queue_size=1)
        self.publisher = rospy.Publisher(
            "/vision/blocksCoords", custMsg, queue_size=1)

    def getT(self, th, c):
        Ry = np.matrix([[cos(th), 0, sin(th), 0], [0, 1, 0, 0],
                       [-sin(th), 0, cos(th), 0], [0, 0, 0, 1]])
        c = np.matrix([[1, 0, 0, -c[0]], [0, 1, 0, -c[1]],
                      [0, 0, 1, -c[2]], [0, 0, 0, 1]])
        return np.dot(Ry, c)

    def unmap(self, val):
        minIn = -180
        maxIn = 493
        minOut = -126
        maxOut = 360
        return (val-minOut)/(maxOut-minOut)*(maxIn-minIn)

    def fromImageToWorld(self, imageCoords):
        # TODO: ignore blocks that are already in their position
        xImage = imageCoords[0]
        yImage = imageCoords[1]
        xl = xImage/self.width-0.5
        yl = self.unmap(yImage-self.height/2-130)/self.height
        th = -3.1415926535/6
        fx = 0.405
        camera = [-0.39, 0.59, 1.39]
        T = self.getT(th, camera)
        pc1_pc0 = -xl/fx
        pc2_pc0 = -yl
        a = (pc2_pc0*0.5977499099999999+0.255073) / \
            (sin(0.523599)-pc2_pc0*cos(0.523599))
        b = 0.59+pc1_pc0*0.5977499099999999+pc1_pc0*cos(0.523599)*a
        return [a, b]

    def getAndSend3dCoords(self, image):
        img = CvBridge().imgmsg_to_cv2(image)
        ris = self.model(img)
        boxes = ris[0].boxes.xyxy
        clss = ris[0].boxes.cls.numpy()
        confs = ris[0].boxes.conf.numpy()
        # Iterate over each found block
        if (len(boxes) > 0):
            arr = boxes[0]
            cx = arr[0]+(arr[2]-arr[0])/2
            cy = arr[1]+(arr[3]-arr[1])/2
            cls = int(clss[0])
            conf = confs[0]
            realWorldCoords = self.fromImageToWorld([cx, cy])
            realWorldCoords.append(self.TAVOLO_HEIGHT)
            rwCoordsParsed = [float(i) for i in realWorldCoords]
            # print(f"This should be an array of 3 items: {realWorldCoords}")
            # print(f"Found block {cls[0]} at coordinates: {realWorldCoords}, with probability: {conf}")
            data = custMsg()
            data.x = rwCoordsParsed[0]
            data.y = rwCoordsParsed[1]
            data.z = rwCoordsParsed[2]
            data.index = cls
            self.publisher.publish(data)
            print(data)
        else:
            print("No blocks found")


if __name__ == "__main__":
    visionNode = VisionNode()
    while not rospy.is_shutdown():
        pass
