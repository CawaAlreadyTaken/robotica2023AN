import os
from gazebo_ros import gazebo_interface
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Quaternion, Point
import rospy
import time
import random
from typing import Iterable
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
from random import randint, shuffle
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import json
from image_to_world import fromImageToWorld

from math import cos, sin, pi
import numpy as np

blocks = [
    ("X1-Y1-Z2", 0),
    ("X1-Y2-Z1", 1),
    ("X1-Y2-Z2-CHAMFER", 2),
    ("X1-Y2-Z2", 3),
    ("X1-Y2-Z2-TWINFILLET", 4),
    ("X1-Y3-Z2-FILLET", 5),
    ("X1-Y3-Z2", 6),
    ("X1-Y4-Z1", 7),
    ("X1-Y4-Z2", 8),
    ("X2-Y2-Z2-FILLET", 9),
    ("X2-Y2-Z2", 10)
]
block_radius = 0.05

# this will be divided by 100
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
        self.set_model_state = rospy.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)
        self.get_model_state = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)
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
                cont += str(block["index"]) + " " + clear(img_pos[0]/1920) + " " + clear(img_pos[1]/1080) + " " + clear((block["cx"]-block["cx"]
                                                                                                                         * 0.5389*(902-img_pos[1])/480)/1920) + " " + clear((block["cy"]-block["cy"]*0.5389*(902-img_pos[1])/480)/1080) + "\n"
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

    def setStatic(self, model_name):
        model_state = ModelState()
        model_state.pose = self.get_model_state(model_name=model_name).pose
        model_state.model_name = model_name
        #model_state.is_static = True
        self.set_model_state(model_state)


spawner = ObjectSpawner()
spawner.__init__()
spawner.updateModels()
rospy.wait_for_service("/gazebo/get_model_state")
# spawner.cleanModels()


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

# while (True):
#     n_blocks = 3
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
#             "orient": [pi/2, pi/2, float(boxes[used_blocks[i][0].split("_")[0]][angle_index]["angle"]) * 2 * pi / 360],
#         })
#     spawner.spawn(blocks_info, static=False)
#     time.sleep(1)
#     spawner.screenShot_and_label(blocks_info)
#     # img_counter += 1
#     spawner.setStatic("ur5")
#     input("Press Enter to continue...")
#     spawner.delete(blocks_info)

block_n = 3

while (True):
    used_blocks = blocks
    blocks_info = []
    name = used_blocks[block_n][0]
    angle_val = random.randint(0,360)
    sizes = boxes[block_n][int((angle_val+45/2)/45)]["size"]
    blocks_info.append({
        "name": name,
        "index": used_blocks[0][1],
        "cx": sizes[0],
        "cy": sizes[1],
        "pos": [0.39, 0.59, 1.39],
        "orient": [random.randint(1,4) * pi/2, 0, angle_val * 2 * pi / 360],
    })
    spawner.spawn(blocks_info, static=True)
    input("Press Enter to continue...")
    spawner.delete(blocks_info)

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
