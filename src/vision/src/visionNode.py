import rospy
import numpy as np
from math import sin, cos
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision.msg import custMsg
from cv2 import imshow, waitKey, resize, imwrite, imread, matchTemplate, TM_CCOEFF_NORMED, minMaxLoc
from os import system, listdir, path
from time import sleep
import random


class VisionNode():
    def __init__(self):
        self.HOME_DIR = "/home/stepo/robotica2023AN/src/vision/src/"
        self.model = YOLO(f"{self.HOME_DIR}best.pt")
        self.TAVOLO_HEIGHT = 0.87
        self.width = 1920
        self.height = 1080
        self.image = None
        rospy.init_node('neuralNetwork')
        rospy.Subscriber("/ur5/zed_node/left/image_rect_color",
                         Image, callback= self.setImage, queue_size=1)
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
    
    def setImage(self, image):
        self.image = image


    def fromImageToWorld(self, imageCoords):
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
    
    def filterRis(self, ris):
        newRis = []
        for r in ris:
                if (r.boxes.conf.numpy()[0] >= 0.55):
                    newRis.append(r)
        return newRis

    def filterBoxes(self, boxes):
        print(boxes)
        newBoxes = []
        for b in boxes:
            cx = b[0]+(b[2]-b[0])/2
            cy = b[1]+(b[3]-b[1])/2
            if (cx >= 678 and cx <= 1538 and cy >= 535 and cy <= 903):
                newBoxes.append(b)
        return newBoxes

    def template_matching(self, img, template_path):
        template = imread(template_path, 0)
        rateoX = template.shape[0]/img.shape[0]
        rateoY = template.shape[1]/img.shape[1]
        if (rateoX >= 1 or rateoY >= 1):
            mass = rateoX if rateoX > rateoY else rateoY
            template = resize(template, ( int(template.shape[1]/mass), int(template.shape[0]/mass) ))
        result = matchTemplate(img, template, TM_CCOEFF_NORMED)
        return minMaxLoc(result)[1]

    def getOrientation(self):
        responses = []
        img = imread(f"{self.HOME_DIR}cropped.jpg", 0)
        for item in listdir(f"{self.HOME_DIR}templates"):
            item_path = path.join(f"{self.HOME_DIR}templates", item)
            responses.append([self.template_matching(img, item_path), item[:-2]])
        responses.sort(reverse=True)
        # print(responses)
        if responses[0][1] == "basso":
            return 0
        elif responses[0][1] == "laterale":
            return 1
        elif responses[0][1] == "alto":
            return 2
        else:
            raise Exception("Errore QUA")
    

    def getAndSend3dCoords(self):
        while(True):
            #input("Press Enter to continue...")
            sleep(1)
            if(self.image is None):
                continue
            img = CvBridge().imgmsg_to_cv2(self.image)
            ris = self.model(img)
            ris = self.filterRis(ris)
            boxes = ris[0].boxes.xyxy
            clss = ris[0].boxes.cls.numpy()
            confs = ris[0].boxes.conf.numpy()
            boxes = self.filterBoxes(boxes)
            # Iterate over each found block
            if (len(boxes) > 0):
            #for i in range (len(boxes)):
                arr = boxes[0]
                cx = arr[0]+(arr[2]-arr[0])/2
                cy = arr[1]+(arr[3]-arr[1])/2
                cropped_image = img[int(arr[1]):int(arr[3]), int(arr[0]):int(arr[2]), 0:3]
                imwrite(f"{self.HOME_DIR}cropped.jpg", cropped_image)
                orientamento = self.getOrientation() # 0 basso, 1 laterale, 2 alto 
                # TODO: parsa meglio (coi gradi) e usa orientamento
                cls = int(clss[0])
                conf = confs[0]
                realWorldCoords = self.fromImageToWorld([cx, cy])
                realWorldCoords.append(self.TAVOLO_HEIGHT)
                rwCoordsParsed = [float(j) for j in realWorldCoords]
                # print(f"This should be an array of 3 items: {realWorldCoords}")
                print(f"Found block {cls} at coordinates: {realWorldCoords}, with probability: {conf}")
                data = custMsg()
                data.x = rwCoordsParsed[0]
                data.y = rwCoordsParsed[1]
                data.z = rwCoordsParsed[2]
                data.orient = orientamento
                data.index = cls
                self.publisher.publish(data)
                print(data)
            else:
                print("No blocks found")


if __name__ == "__main__":
    visionNode = VisionNode()
    visionNode.getAndSend3dCoords()
    while not rospy.is_shutdown():
        pass
