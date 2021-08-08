
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped 
import tf

current_pose_topic = "/initialpose"

def toRPY(pose):
    return tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])


def pose_callback(data=PoseWithCovarianceStamped):
    p = data.pose.pose.position
    x, y = p.x, p.y
    _, _, Y = toRPY(data.pose.pose)
    return x, y, Y

if __name__ == "__main__":
    rospy.init_node("demo_slam")
    
    while not rospy.is_shutdown():
        pose = rospy.wait_for_message(current_pose_topic, PoseWithCovarianceStamped, 2)
        current_pose = pose_callback(pose)
        print(current_pose)




        
        


