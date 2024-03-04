#!/usr/bin/env python3
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
from std_msgs.msg import Header
import rospy
import numpy as np
from cv_bridge import CvBridge
import message_filters
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_py as tf2

def depth_to_pointcloud(depth_map, fx, fy, cx, cy):

    h, w = depth_map.shape
    # Generate 2D grid of pixel coordinates
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    # Compute 3D coordinates for each pixel
    X = (u - cx) * depth_map / fx
    Y = (v - cy) * depth_map / fy
    Z = depth_map

    # Stack the 3D coordinates to form the point cloud
    point_cloud = np.stack((X, Y, Z), axis=-1)

    # Reshape the point cloud to a 2D array
    point_cloud = point_cloud.reshape(-1, 3)
    return point_cloud

def convertCloudFromOpen3dToRos(points, frame_id="odom", color=1):
        # Set "header"
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id

        # Set "fields" and "cloud_data"
        points = np.asarray(points, dtype=object)
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1)]
        pointcloud_pc2 = pc2.create_cloud(header, fields, points)
        return pointcloud_pc2

class DepthPointcloudNode:
    def __init__(self):
        rospy.init_node('depth_pcd_node')
        
        self.depth_image = message_filters.Subscriber(
            '/depth/image',
            Image
        )
        self.depth_info = message_filters.Subscriber(
            '/depth/info',
            CameraInfo
        )

        self.ts_zed = message_filters.TimeSynchronizer(
            [self.depth_image, self.depth_info], 10)
        self.ts_zed.registerCallback(self.callback)
        
        self.br = CvBridge()

        self.pc_pub = rospy.Publisher(
            '/depth/points', PointCloud2, queue_size=10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer)

    def callback(self, depth_msg, info_msg):
        
        lookup_time = depth_msg.header.stamp + rospy.Duration(0.1)
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link",  depth_msg.header.frame_id,   depth_msg.header.stamp, )
        except tf2.LookupException as ex:
            rospy.logwarn(str(lookup_time.to_sec()))
            rospy.logwarn(ex)
            return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(str(lookup_time.to_sec()))
            rospy.logwarn(ex)
            return
        depth = self.br.imgmsg_to_cv2(
            depth_msg, '32FC1') 
        fx =  info_msg.K[0]
        fy =  info_msg.K[4]
        cx =  info_msg.K[2]
        cy =  info_msg.K[5]

        depth[depth < 0.15] = 10000
        points = depth_to_pointcloud(depth[::5, ::5]/1000, fx/5, fy/5, cx/5, cy/5)

        pcd_pc2 = convertCloudFromOpen3dToRos(
             points, 
             frame_id="realsense_gripper_aligned_depth_to_color_frame", 
             color=1
        )

        point_cloud_ros_tf = do_transform_cloud(pcd_pc2, transform)
        self.pc_pub.publish(point_cloud_ros_tf)

def main(args=None):
    node = DepthPointcloudNode()
    rospy.spin()


if __name__ == '__main__':
    main()
