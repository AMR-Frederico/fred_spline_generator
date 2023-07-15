#!/usr/bin/env python3
from spline_source.spline.spline import Spline2D
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Polygon, Point32, PoseStamped, PoseArray, Pose
from nav_msgs.msg import Path
import tf
import math

class Generator:

    def __init__(self):

        self.spline = Spline2D()
        self.pub_points = rospy.Publisher("spline_generator/out/points", Polygon, queue_size=10)
        self.pub_ctrl_points_pose = rospy.Publisher("spline_generator/out/ctrl_points_pose", PoseArray, queue_size=10)
        self.pub_path = rospy.Publisher("spline_generator/out/path", Path, queue_size=10)
        self.sub_cmd_generate_spline = rospy.Subscriber('spline_generator/cmd/generate_spline', Bool, self.generate_spline)
       

    def generate_spline(self, msg):

        # load from rosparam
        ctrl_points = rospy.get_param("/fred_spline_generator/ctrl_points")
        curve_resolution = rospy.get_param("/fred_spline_generator/config/curve/resolution")
        curve_precision = rospy.get_param("/fred_spline_generator/config/curve/precision")

        self.spline.set(ctrl_points, curve_resolution, curve_precision)
        self.spline.calculate()

        self.publish_points()
        self.publish_path()
        self.publish_ctrl_points_pose(ctrl_points)



    def spline_points_to_ROS_polygon(self, spoints):

        p = Point32()
        pol = Polygon()

        for sp in spoints:

            p.x = sp[0]
            p.y = sp[1]
            p.z = 0

            pol.points.append(p)

        return pol
    


    def publish_ctrl_points_pose(self, points):

        arr = PoseArray()

        arr.header.frame_id = "map"
        arr.header.stamp = rospy.Time.now()

        for sp in points:

            p = Pose()

            p.position.x = sp[0]
            p.position.y = sp[1]
            p.position.z = 0

            quat = tf.transformations.quaternion_from_euler(
                0, 0, -math.radians(0))

            p.orientation.x = quat[0]
            p.orientation.y = quat[1]
            p.orientation.z = quat[2]
            p.orientation.w = quat[3]



            arr.poses.append(p)


        self.pub_ctrl_points_pose.publish(arr)
    


    def publish_points(self):


        pol = self.spline_points_to_ROS_polygon(self.spline.points_spline)
        
        msg = pol

        self.pub_points.publish(msg)
    


    def spline_points_to_ROS_path(self, spoints):

        path = Path()
        seq = 0

        for sp in spoints:

            p = PoseStamped()

            p.pose.position.x = sp[0]
            p.pose.position.y = sp[1]
            p.pose.position.z = 0

            quat = tf.transformations.quaternion_from_euler(
                0, 0, -math.radians(0))

            p.pose.orientation.x = quat[0]
            p.pose.orientation.y = quat[1]
            p.pose.orientation.z = quat[2]
            p.pose.orientation.w = quat[3]

            p.header.stamp = rospy.Time.now()
            p.header.seq = seq
            seq = seq+1


            path.poses.append(p)
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"

        return path


    def publish_path(self):


        pol = self.spline_points_to_ROS_path(self.spline.points_spline)
        
        msg = pol

        self.pub_path.publish(msg)
    


if __name__ == "__main__":

    rospy.init_node('fred_spline_generator')
    rate = rospy.Rate(0.5)
    
    generator = Generator()
    generator.generate_spline(True)

    while not rospy.is_shutdown():
            generator.publish_path()
            
            rate.sleep()

    
    

   
