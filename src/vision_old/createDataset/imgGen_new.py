import os
from gazebo_ros import gazebo_interface
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Quaternion, Point
import rospy
import time
from typing import Iterable
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.msg import ModelStates
from random import randint, shuffle
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import json
from image_to_world import fromImageToWorld

from math import cos, sin, pi
import numpy as np

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
block_radius = 0.05

#this will be divided by 100
MIN_X = 5
MAX_X = 35
MIN_Y = 20
MAX_Y = 75
H = 87

# MIN_X = 5
# MAX_X = 95
# MIN_Y = 20
# MAX_Y = 75
# H = 87

img_msg = None

img_counter = 4710

boxes = {}
with open("boxes.json", "r") as f:
    boxes = json.loads(f.read())


def callback(img_msg_):
    global img_msg
    img_msg = img_msg_

def clear(val):
    return str(val).replace("[", "").replace("]", "")


class ObjectSpawner(object):
    def __init__(self):
        rospy.init_node("spawn_stuff_and_get_images")
        rospy.wait_for_service('/gazebo/delete_model')
        rospy.Subscriber('/gazebo/model_states',
                         ModelStates, self.getModelsNames)
        rospy.Subscriber("/ur5/zed_node/left/image_rect_color",
                         Image,   callback=callback, queue_size=1)
        self._delete_model = rospy.ServiceProxy(
            '/gazebo/delete_model', DeleteModel)
        self.gazebo_models = []

    def updateModels(self):
        print("waiting forcamera and models to update")
        while (len(self.gazebo_models) == 0 and img_msg is None):
            time.sleep(1)
        print("camera online and models updated")

    def getModelsNames(self, model_states):
        if (len(model_states.name) > 0):
            self.gazebo_models = model_states.name

    def screenShot_and_label(self, blocks_info):
        global img_counter
        img = CvBridge().imgmsg_to_cv2(img_msg)
        print("saving image: ", img_counter)
        cv2.imwrite("./dataset/imgs/" + str(img_counter) + ".jpg", img)
        with open("./dataset/labels/" + str(img_counter) + ".txt", "w") as f:
            cont = ""
            for block in blocks_info:
                print("real world pos: ", block["pos"])
                img_pos = self.fromWorldToImage(
                    np.matrix([[block["pos"][0]], [block["pos"][1]], [block["pos"][2]], [1]]))
                cont += str(block["index"]) + " " + clear(img_pos[0]/1920) + " " + clear(img_pos[1]/1080) + " " + clear((block["cx"]-block["cx"]*0.5389*(902-img_pos[1])/480)/1920) + " " + clear((block["cy"]-block["cy"]*0.5389*(902-img_pos[1])/480)/1080) + "\n"
            f.write(cont)

    def randPos(self):
        x = randint(MIN_X, MAX_X)
        y = randint(MIN_Y, MAX_Y)
        return [x/100, y/100, H/100]

    def delete(self, blocks_info):
        for block in blocks_info:
            for model in self.gazebo_models:
                if (block["name"] == model.split("_")[0]):
                    try:
                        rospy.loginfo("Delete model: %s" % model)
                        self._delete_model(model)
                    except rospy.ServiceException as e:
                        rospy.logerr("Delete model failed: %s" % e.message)

    def spawn(self, blocks_info, static=False):
        for block in blocks_info:
            model_database_template = """<sdf version="1.4">
                    <world name="default">
                        <include>
                        <uri>model://%MODEL_NAME%</uri>
                        <static>%STATIC%</static>
                        </include>
                    </world>
                    </sdf>"""
            model_xml = model_database_template \
                .replace('%MODEL_NAME%', block["name"]) \
                .replace('%STATIC%', str(int(static)))
            initial_pose = Pose(Point(*block["pos"]),
                                Quaternion(*quaternion_from_euler(*block["orient"])))
            gazebo_model_name = "%s_%d" % (
                block["name"], round(time.time() * 1000))
            gazebo_interface.spawn_sdf_model_client(
                gazebo_model_name, model_xml, rospy.get_namespace(), initial_pose, "", "/gazebo")
            rospy.loginfo("%s spawned in Gazebo as %s",
                          block["name"], gazebo_model_name)
        return gazebo_model_name

    def _getT(self, th, c):
        Ry = np.matrix([[cos(th), 0, sin(th), 0], [0, 1, 0, 0],
                       [-sin(th), 0, cos(th), 0], [0, 0, 0, 1]])
        c = np.matrix([[1, 0, 0, -c[0]], [0, 1, 0, -c[1]],
                      [0, 0, 1, -c[2]], [0, 0, 0, 1]])
        return np.dot(Ry, c)

    def _map(self, val):
        minIn = -180
        maxIn = 493
        minOut = -126
        maxOut = 360
        return (val/(maxIn-minIn))*(maxOut-minOut) + minOut

    def fromWorldToImage(self, Pw):
        th = -3.1415926535/6
        fx = 0.405
        camera = [-0.39, 0.59, 1.39]
        T = self._getT(th, camera)
        Pc = np.dot(T, Pw)
        xl = -fx*Pc[1]/Pc[0]
        yl = -Pc[2]/Pc[0]
        width = 1920
        height = 1080
        return [width/2+(xl*width), height/2+self._map(yl*height)+122]

    def cleanModels(self):
        for model in self.gazebo_models:
            if (model[0] == "X"):
                try:
                    self._delete_model(model)
                except:
                    pass


spawner = ObjectSpawner()
spawner.__init__()
spawner.updateModels()
#spawner.cleanModels()


# while(True):
#     n_blocks = randint(1, 5)
#     used_blocks = blocks
#     shuffle(used_blocks)
#     blocks_info = []
#     for i in range(n_blocks):
#         angle_index = int(used_blocks[i][1]/11)
#         name = used_blocks[i][0].split("_")[0]
#         blocks_info.append({
#             "name": name,
#             "index": used_blocks[i][1],
#             "cx": boxes[name][angle_index]["size"][0],
#             "cy": boxes[name][angle_index]["size"][1],
#             "pos": spawner.randPos(),
#             "orient": [0, 0, float(boxes[used_blocks[i][0].split("_")[0]][angle_index]["angle"]) * 2 * pi / 360],
#         })
#     spawner.spawn(blocks_info, static=False)
#     time.sleep(1)
#     spawner.screenShot_and_label(blocks_info)
#     img_counter += 1
#     spawner.delete(blocks_info)

while(True):
    n_blocks = 3
    used_blocks = blocks
    shuffle(used_blocks)
    blocks_info = []
    for i in range(n_blocks):
        angle_index = int(used_blocks[i][1]/11)
        name = used_blocks[i][0].split("_")[0]
        blocks_info.append({
            "name": name,
            "index": used_blocks[i][1],
            "cx": boxes[name][angle_index]["size"][0],
            "cy": boxes[name][angle_index]["size"][1],
            "pos": spawner.randPos(),
            "orient": [0, 0, float(boxes[used_blocks[i][0].split("_")[0]][angle_index]["angle"]) * 2 * pi / 360],
        })
    spawner.spawn(blocks_info, static=False)
    time.sleep(1)
    spawner.screenShot_and_label(blocks_info)
    #img_counter += 1
    input("Press Enter to continue...")
    spawner.delete(blocks_info)

# spawner.screenShot_and_label([{
#     "name": "block_1",
#     "index": 0,
#     "cx": 0.1,
#     "cy": 0.1,
#     "pos": [0.5, 0.5, 0.5],
#     "orient": [0, 0, 0]
#     }])
    


# spawned = []

# while (True):
#     n_blocks = randint(1, 5)
#     used_blocks = blocks
#     shuffle(used_blocks)
#     blocks_info = []
#     print("#################################################################")
#     for i in range(n_blocks):
#         pos = spawner.randPos()
#         blocks_info.append({
#             "name": used_blocks[i][0],
#             "index": used_blocks[i][1],
#             "cx": used_blocks[i][2],
#             "cy": used_blocks[i][3],
#             "pos": pos,
#             "orient": [0, 0, 0]
#         })
        
#         print("block " + used_blocks[i][0] + " should spawns at: " + str(pos))
#         wi = spawner.fromWorldToImage([[pos[0]], [pos[1]], [pos[2]], [1]])
#         wi_parsed = [int(float(str(wi[0]).replace("[", "").replace("]", ""))), int(
#             float(str(wi[1]).replace("[", "").replace("]", "")))]
#         print("in the image should be at: " + str(wi_parsed))
#         print("calculated world position: " +
#               str(fromImageToWorld(wi_parsed)))
#         print("\n")
#     print("#################################################################")
#     spawner.spawn(blocks_info, static=False)
#     time.sleep(2)
#     spawner.screenShot_and_label(blocks_info)
#     spawned.append(blocks_info)
#     if (input("Press Enter to continue...") == "q"):
#         break

# spawner.delete(spawned)


# block scan
#
# for block in blocks:
#     b = {
#         "name": block[0],
#         "index": block[1],
#         "pos": [0.02, 0.4, 0.87],
#         "orient": [0, 0, 0],
#         "cx": block[2],
#         "cy": block[3]
#     }
#     spawner.spawn([b], static=True)
#     time.sleep(2)
#     spawner.screenShot_and_label([b])
#     input("Press Enter to continue...")
#     spawner.delete([b])

# spawner.spawn([{
#     "name": "X1-Y1-Z2",
#     "index": 0,
#     "pos": [-0.39, 0.59, 1.39],
#     "orient": [0, 0, 0]
# }], static=True)
# spawner.spawn([{
#     "name": "X1-Y1-Z2",
#     "index": 0,
#     "pos": [0., 0., 0.],
#     "orient": [0, 0, 0]
# }], static=True)

# #angolo in basso a sinistra
# spawner.spawn([{
#     "name": "X1-Y1-Z2",
#     "index": 0,
#     "pos": [-0.02, 0.8, H/100],
#     "orient": [0, 0, 0]
# }], static=True)

# #angolo in alto a sinistra
# spawner.spawn([{
#     "name": "X1-Y1-Z2",
#     "index": 0,
#     "pos": [1, 0.8, H/100],
#     "orient": [0, 0, 0]
# }], static=True)

# #angolo in alto a destra
# spawner.spawn([{
#     "name": "X1-Y1-Z2",
#     "index": 0,
#     "pos": [1, 0.16, H/100],
#     "orient": [0, 0, 0]
# }], static=True)

# spawner.spawn([{
#     "name": "X1-Y1-Z2",
#     "index": 0,
#     "pos": [0.1, 0.23, H/100],
#     "orient": [0, 0, 0]
# }], static=True)

# spawner.spawn([{
#     "name": "X1-Y1-Z2",
#     "index": 0,
#     "pos": [0.259, 0.371, 0.87],
#     "orient": [0, 0, 0],
#     "cx": 0,
#     "cy": 0
# }], static=True)
# time.sleep(2)
# spawner.screenShot_and_label([{
#     "name": "X1-Y1-Z2",
#     "index": 0,
#     "pos": [0.119, 0.444, 0.87],
#     "orient": [0, 0, 0],
#     "cx": 0,
#     "cy": 0
# }])