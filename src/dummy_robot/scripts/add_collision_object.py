import rospy
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

rospy.init_node('add_collision_object_node')

planning_scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene, queue_size=10)
rospy.sleep(1)  # Give time for the publisher to connect

# Define a box collision object
collision_object = CollisionObject()
collision_object.id = "obstacle"
collision_object.header.frame_id = "world"
collision_object.operation = CollisionObject.ADD

box = SolidPrimitive()
box.type = SolidPrimitive.BOX
box.dimensions = [0.5, 0.5, 0.5]  # size: x, y, z

# Define the pose of the box (position + orientation)
pose = Pose()
pose.position.x = 1.0
pose.position.y = 1.0
pose.position.z = 0.25  # Place half above the "ground"

collision_object.primitives = [box]
collision_object.primitive_poses = [pose]

# Add the collision object to the planning scene
planning_scene = PlanningScene()
planning_scene.world.collision_objects.append(collision_object)
planning_scene.is_diff = True
planning_scene_pub.publish(planning_scene)

