import os
from gazebo_ros import gazebo_interface
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Quaternion, Point
import rospy
import time
from typing import Iterable
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.msg import ModelStates


class ObjectSpawner(object):
    def __init__(self):
        rospy.init_node("spawn_stuff_and_get_images")
        rospy.wait_for_service('/gazebo/delete_model')
        rospy.Subscriber('/gazebo/model_states',
                         ModelStates, self.getModelsNames)
        self._delete_model = rospy.ServiceProxy(
            '/gazebo/delete_model', DeleteModel)
        self.gazebo_models = []

    def updateModels(self):
        print("waiting for models to update")
        while (len(self.gazebo_models) == 0):
            time.sleep(1)
        print("models updated")

    def getModelsNames(self, model_states):
        if (len(model_states.name) > 0):
            self.gazebo_models = model_states.name

    def delete(self, model_name):
        for model in self.gazebo_models:
            if (model_name == model.split("_")[0]):
                try:
                    rospy.loginfo("Delete model: %s" % model)
                    self._delete_model(model)
                except rospy.ServiceException as e:
                    rospy.logerr("Delete model failed: %s" % e.message)

    def spawn(self, model_name, positions, orientations, static=False):
        # type: (str, Iterable[float], Iterable[float],bool) -> str
        model_database_template = """<sdf version="1.4">
                  <world name="default">
                    <include>
                      <uri>model://%MODEL_NAME%</uri>
                      <static>%STATIC%</static>
                    </include>
                  </world>
                </sdf>"""
        model_xml = model_database_template \
            .replace('%MODEL_NAME%', model_name) \
            .replace('%STATIC%', str(int(static)))
        initial_pose = Pose(Point(*positions),
                            Quaternion(*quaternion_from_euler(*orientations)))
        gazebo_model_name = "%s_%d" % (model_name, round(time.time() * 1000))
        gazebo_interface.spawn_sdf_model_client(gazebo_model_name, model_xml, rospy.get_namespace(),
                                                initial_pose, "", "/gazebo")
        rospy.loginfo("%s spawned in Gazebo as %s",
                      model_name, gazebo_model_name)
        return gazebo_model_name


spawner = ObjectSpawner()
spawner.__init__()
spawner.updateModels()


while (True):
    loc = input("inserisci la location: (# # #)")
    loc = loc.split(" ")
    ObjectSpawner.spawn(spawner, "X1-Y1-Z2",[float(loc[0]), float(loc[1]), float(loc[2])], [0, 0, 0], static=False)
    input("Press Enter to cdelete...")
    spawner.delete("X1-Y1-Z2")
