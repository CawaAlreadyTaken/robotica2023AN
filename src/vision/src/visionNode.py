import rospy
import numpy as np
from math import sin, cos, sqrt
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision.msg import custMsg
from cv2 import imshow, waitKey, resize
from os import system

blocks = [
    ("X1-Y1-Z2_0", 0),
    ("X1-Y2-Z1_0", 1),
    ("X1-Y2-Z2-CHAMFER_0", 2),
    ("X1-Y2-Z2_0", 3),
    ("X1-Y2-Z2-TWINFILLET_0", 4),
    ("X1-Y3-Z2-FILLET_0", 5),
    ("X1-Y3-Z2_0", 6),
    ("X1-Y4-Z1_0", 7),
    ("X1-Y4-Z2_0", 8),
    ("X2-Y2-Z2-FILLET_0", 9),
    ("X2-Y2-Z2_0", 10),
    ("X1-Y1-Z2_45", 11),
    ("X1-Y2-Z1_45", 12),
    ("X1-Y2-Z2-CHAMFER_45", 13),
    ("X1-Y2-Z2_45", 14),
    ("X1-Y2-Z2-TWINFILLET_45", 15),
    ("X1-Y3-Z2-FILLET_45", 16),
    ("X1-Y3-Z2_45", 17),
    ("X1-Y4-Z1_45", 18),
    ("X1-Y4-Z2_45", 19),
    ("X2-Y2-Z2-FILLET_45", 20),
    ("X2-Y2-Z2_45", 21),
    ("X1-Y1-Z2_90", 22),
    ("X1-Y2-Z1_90", 23),
    ("X1-Y2-Z2-CHAMFER_90", 24),
    ("X1-Y2-Z2_90", 25),
    ("X1-Y2-Z2-TWINFILLET_90", 26),
    ("X1-Y3-Z2-FILLET_90", 27),
    ("X1-Y3-Z2_90", 28),
    ("X1-Y4-Z1_90", 29),
    ("X1-Y4-Z2_90", 30),
    ("X2-Y2-Z2-FILLET_90", 31),
    ("X2-Y2-Z2_90", 32),
    ("X1-Y1-Z2_135", 33),
    ("X1-Y2-Z1_135", 34),
    ("X1-Y2-Z2-CHAMFER_135", 35),
    ("X1-Y2-Z2_135", 36),
    ("X1-Y2-Z2-TWINFILLET_135", 37),
    ("X1-Y3-Z2-FILLET_135", 38),
    ("X1-Y3-Z2_135", 39),
    ("X1-Y4-Z1_135", 40),
    ("X1-Y4-Z2_135", 41),
    ("X2-Y2-Z2-FILLET_135", 42),
    ("X2-Y2-Z2_135", 43),
    ("X1-Y1-Z2_180", 44),
    ("X1-Y2-Z1_180", 45),
    ("X1-Y2-Z2-CHAMFER_180", 46),
    ("X1-Y2-Z2_180", 47),
    ("X1-Y2-Z2-TWINFILLET_180", 48),
    ("X1-Y3-Z2-FILLET_180", 49),
    ("X1-Y3-Z2_180", 50),
    ("X1-Y4-Z1_180", 51),
    ("X1-Y4-Z2_180", 52),
    ("X2-Y2-Z2-FILLET_180", 53),
    ("X2-Y2-Z2_180", 54),
    ("X1-Y1-Z2_225", 55),
    ("X1-Y2-Z1_225", 56),
    ("X1-Y2-Z2-CHAMFER_225", 57),
    ("X1-Y2-Z2_225", 58),
    ("X1-Y2-Z2-TWINFILLET_225", 59),
    ("X1-Y3-Z2-FILLET_225", 60),
    ("X1-Y3-Z2_225", 61),
    ("X1-Y4-Z1_225", 62),
    ("X1-Y4-Z2_225", 63),
    ("X2-Y2-Z2-FILLET_225", 64),
    ("X2-Y2-Z2_225", 65),
    ("X1-Y1-Z2_270", 66),
    ("X1-Y2-Z1_270", 67),
    ("X1-Y2-Z2-CHAMFER_270", 68),
    ("X1-Y2-Z2_270", 69),
    ("X1-Y2-Z2-TWINFILLET_270", 70),
    ("X1-Y3-Z2-FILLET_270", 71),
    ("X1-Y3-Z2_270", 72),
    ("X1-Y4-Z1_270", 73),
    ("X1-Y4-Z2_270", 74),
    ("X2-Y2-Z2-FILLET_270", 75),
    ("X2-Y2-Z2_270", 76),
    ("X1-Y1-Z2_315", 77),
    ("X1-Y2-Z1_315", 78),
    ("X1-Y2-Z2-CHAMFER_315", 79),
    ("X1-Y2-Z2_315", 80),
    ("X1-Y2-Z2-TWINFILLET_315", 81),
    ("X1-Y3-Z2-FILLET_315", 82),
    ("X1-Y3-Z2_315", 83),
    ("X1-Y4-Z1_315", 84),
    ("X1-Y4-Z2_315", 85),
    ("X2-Y2-Z2-FILLET_315", 86),
    ("X2-Y2-Z2_315", 87),
]

class VisionNode():
    def __init__(self):
        self.model = YOLO("weights.pt")
        self.TAVOLO_HEIGHT = 0.87
        self.width = 1920
        self.height = 1080
        self.image = None
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
    
    def filterBoxes(self, boxes):
        newBoxes = []
        for b in boxes:
            cx = b[0]+(b[2]-b[0])/2
            cy = b[1]+(b[3]-b[1])/2
            if (cx >= 678 and cx <= 1538 and cy >= 561 and cy <= 903):
                newBoxes.append(b)
            else:
                newBoxes.append(None)
        return newBoxes
    
    def printRes(self,results):
        res_arr = []
        for ris in results[0].numpy():
            res_arr.append(list(dict.fromkeys(str(ris).replace("[","").replace("]","").split(" ")))[1:])
        for res in res_arr:
            cx = float(res[0])+(float(res[2])-float(res[0]))/2
            cy = float(res[1])+(float(res[3])-float(res[1]))/2
            print(f"class: {blocks[int(res[5])]}, confidence: {res[4]} , x: {cx}, y: {cy}")

    def getAndSend3dCoords(self,image):
        img = CvBridge().imgmsg_to_cv2(image)
        ris = self.model(img)
        print("#############################################")
        self.printRes(ris)
        print("#############################################")
        boxes = ris[0].boxes.xyxy
        clss = ris[0].boxes.cls.numpy()
        confs = ris[0].boxes.conf.numpy()
        boxes = self.filterBoxes(boxes)
        if (len(boxes) > 0):
            box_index = -1
            for i in range(len(boxes)):
                if(boxes[i] is not None):
                    box_index = i
                    break
            if(box_index == -1):
                print("No box found")
                return
            arr = boxes[box_index]
            cx = arr[0]+(arr[2]-arr[0])/2
            cy = arr[1]+(arr[3]-arr[1])/2
            cls = int(clss[box_index])
            conf = confs[box_index]
            realWorldCoords = self.fromImageToWorld([cx, cy])
            realWorldCoords.append(self.TAVOLO_HEIGHT)
            rwCoordsParsed = [float(i) for i in realWorldCoords]
            # print(f"This should be an array of 3 items: {realWorldCoords}")
            print(f"Found block {cls} at coordinates: {realWorldCoords}, with probability: {conf}")
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
