import rospy as ros
import message_filters
import copy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from gazebo_msgs.srv import DeleteModel, SpawnModel
import tf
from geometry_msgs.msg import Pose, Point, Quaternion


def callback(img_msg):
    img = CvBridge().imgmsg_to_cv2(img_msg)
    cv2.imwrite("bella.jpg", img)
    cv2.imshow("bella", img)
    cv2.waitKey(1)


def main():
    ros.init_node("spawn_stuff_and_get_images")
    print("Waiting for service")
    ros.Subscriber(
       "/ur5/zed_node/left/image_rect_color", Image,   callback=callback, queue_size=1)
    # ros.wait_for_service("/gazebo/delete_model")
    # ros.wait_for_service("/gazebo/spawn_model")
    print("Service is up")
    while(True):
        pass
    # delete_model = ros.ServiceProxy("gazebo/delete_model", DeleteModel)
    # spawn_model = ros.ServiceProxy("gazebo/spawn_model", SpawnModel)

    product_xml = f"""<?xml version="1.0" ?>
<sdf version="1.4">

<physics type="ode">

      <ode>
 
        <constraints>
          <contact_max_correcting_vel>0.1</contact_max_correcting_vel>
          <contact_surface_layer>0.0001</contact_surface_layer>
        </constraints>
      	</ode>
	<max_contacts>10</max_contacts>
	</physics>

<model name="X1-Y1-Z2">
	<link name="link">
	<inertial>
			<mass>4.0</mass>
			<inertia> <!-- inertias are tricky to compute -->
			  <!-- http://gazebosim.org/tutorials?tut=inertia&cat=build_robot -->
			  <ixx>0.083</ixx>       <!-- for a box: ixx = 0.083 * mass * (y*y + z*z) -->
			  <ixy>0.0</ixy>         <!-- for a box: ixy = 0 -->
			  <ixz>0.0</ixz>         <!-- for a box: ixz = 0 -->
			  <iyy>0.083</iyy>       <!-- for a box: iyy = 0.083 * mass * (x*x + z*z) -->
			  <iyz>0.0</iyz>         <!-- for a box: iyz = 0 -->
			  <izz>0.083</izz>       <!-- for a box: izz = 0.083 * mass * (x*x + y*y) -->
			</inertia>
		      </inertial>
		<collision name="collision">
		
			<geometry>
				<mesh>
					<uri>model://X1-Y1-Z2/mesh/X1-Y1-Z2.stl</uri>
					<scale>1 1 1</scale>
				</mesh>
				
		    	       
			</geometry>
		</collision>
			   
		<visual name="visual">
		<material>  
			  <ambient>0.1 0.1 0.1 1</ambient>
			  <diffuse>0.1 0.1 0.2 1</diffuse>
			  <specular>0 0 0 0</specular>
			  <emissive>0 0 0 1</emissive>
			</material> 
			<geometry>
			  	<mesh>
					<uri>model://X1-Y1-Z2/mesh/X1-Y1-Z2.stl</uri>
					<scale>1 1 1</scale>
				</mesh>
				
			   
			</geometry>
		</visual>
		<mu1>10</mu1>
			    <mu2>10</mu2>
			  
			    <kp>100000.0</kp>
			    <kd>10.0</kd> 
			    <fdir1>0 0 1</fdir1>
			    <minDepth>0.001</minDepth>
			    <maxVel>0.0</maxVel>
			    <maxContacts>2</maxContacts>
			    <material>Gazebo/RedBright</material>
	</link>
</model>
</sdf>"""

    for i in range(10):
        print("Spawning model:%s", "X1-Y1-Z2")
        orient = Quaternion(tf.transformations.quaternion_from_euler(0, 0, 0))

        for num in xrange(0, 12):
            item_name = "product_{0}_0".format(num)
            print("Deleting model:%s", item_name)
            delete_model(item_name)

        for num in xrange(0, 12):
            bin_y = 2.8 * (num / 6) - 1.4
            bin_x = 0.5 * (num % 6) - 1.5
            item_name = "product_{0}_0".format(num)
            print("Spawning model:%s", item_name)
            item_pose = Pose(Point(x=bin_x, y=bin_y,    z=2),   orient)
            spawn_model(item_name, product_xml, "", item_pose, "world")


if __name__ == "__main__":
    main()
