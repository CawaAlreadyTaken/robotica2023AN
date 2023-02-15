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
from math import pi

blocks = [
    ("X1-Y1-Z2", 0,),
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
MAX_X = 95
MIN_Y = 20
MAX_Y = 75
H = 87

img_msg = None

img_counter = 0


def callback(img_msg_):
    global img_msg
    img_msg = img_msg_


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
        cv2.imwrite("./dataset/imgs/" + str(img_counter) + ".jpg", img)
        with open("./dataset/labels/" + str(img_counter) + ".txt", "w") as f:
            cont = ""
            for block in blocks_info:
                cont += str(block["index"]) + " " + str(block["pos"][0]) + " " + str(block["pos"][1]) + " " + "0.03" + " " + "0.03" + "\n"
            f.write(cont)
        img_counter += 1

    def randPos(self):
        x = randint(MIN_X, MAX_X)
        y = randint(MIN_Y, MAX_Y)
        return [x/100, y/100, H/100]

    def delete(self, blocks_info):
        for block in blocks_info:
            for model in self.gazebo_models:
                if (block["name"]== model.split("_")[0]):
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
            gazebo_interface.spawn_sdf_model_client(gazebo_model_name, model_xml, rospy.get_namespace(),initial_pose, "", "/gazebo")
            rospy.loginfo("%s spawned in Gazebo as %s",
                          block["name"], gazebo_model_name)
        return gazebo_model_name


spawner = ObjectSpawner()
spawner.__init__()
spawner.updateModels()



# while (True):
#     n_blocks = randint(1, 5)
#     used_blocks = blocks
#     shuffle(used_blocks)
#     blocks_info = []
#     for i in range(n_blocks):
#         blocks_info.append({
#             "name": used_blocks[i][0],
#             "index": used_blocks[i][1],
#             "pos": spawner.randPos(),
#             "orient": [0, 0, 0]
#         })
#     ObjectSpawner.spawn(spawner, blocks_info, static=False)
#     time.sleep(2)

#     spawner.screenShot_and_label(blocks_info)
#     spawner.delete(blocks_info)

def delete_file(filename):
    if os.path.exists(filename):
        os.remove(filename)

for i in blocks:
    for angle in range(8):
        blocks_info = []
        blocks_info.append({
            "name": i[0],
            "index": i[1],
            "pos": [0.05, 0.5, H/100],
            "orient": [0, 0, angle * 2 * pi / 8]
        })
        ObjectSpawner.spawn(spawner, blocks_info, static=False)
        time.sleep(2)

        spawner.screenShot_and_label(blocks_info)
        input("Press Enter to continue...")
        delete_file("./dataset/imgs/0.jpg")
        delete_file("./dataset/labels/0.txt")
        img_counter = 0
        spawner.delete(blocks_info)

