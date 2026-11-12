"""ROS adapter: synchronise RGB/depth, transform LiDAR and publish detections."""
import json
import time
import numpy as np
import rclpy
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker,MarkerArray
from .world import load_config
from .perception_core import sensor_transform,detect,Stability


def stamp(message):
    return message.header.stamp.sec+message.header.stamp.nanosec*1e-9


class Perception(Node):
    def __init__(self):
        super().__init__('factory_perception')
        self.declare_parameter('config',str(Path(get_package_share_directory('pieas_sorting'))/'config/cell.yaml'))
        self.c=load_config(self.get_parameter('config').value)
        self.bridge=CvBridge();self.rgb=None;self.depth=None;self.info=None;self.cloud=None
        self.last_rgb_stamp=-1.;self.last_warning=0.;self.last_cloud_stamp=-1.
        self.stability=Stability(self.c['perception']['stable_frames'])
        self.output=self.create_publisher(String,'/factory/detections',10)
        self.debug=self.create_publisher(Image,'/factory/perception_mask',qos_profile_sensor_data)
        self.markers=self.create_publisher(MarkerArray,'/factory/detection_markers',10)
        for kind,topic,attribute in [(Image,'/factory/camera/image','rgb'),
                                    (Image,'/factory/camera/depth_image','depth'),
                                    (CameraInfo,'/factory/camera/camera_info','info'),
                                    (PointCloud2,'/factory/lidar/points','cloud')]:
            self.create_subscription(kind,topic,lambda m,a=attribute:setattr(self,a,m),qos_profile_sensor_data)
        self.create_timer(0.05,self.tick)

    def publish(self,detections,reason,when):
        self.output.publish(String(data=json.dumps({'stamp':when,'detections':detections,'reason':reason},allow_nan=False)))

    def tick(self):
        now=self.get_clock().now().nanoseconds*1e-9
        if any(x is None for x in [self.rgb,self.depth,self.info,self.cloud]):
            self.publish([],'waiting_for_rgb_depth_camera_info_lidar',now);return
        if any(not 0<=now-stamp(m)<0.5 for m in [self.rgb,self.depth,self.cloud]):
            self.stability.update([]);self.publish([],'stale_sensor_data',now);return
        if abs(stamp(self.rgb)-stamp(self.depth))>0.06 or abs(stamp(self.rgb)-stamp(self.cloud))>0.15:
            self.stability.update([]);self.publish([],'sensor_timestamp_mismatch',now);return
        # Do not count a repeated frame as another independent observation.
        if stamp(self.rgb)==self.last_rgb_stamp or stamp(self.cloud)==self.last_cloud_stamp:return
        if self.last_rgb_stamp>stamp(self.rgb):self.stability.update([])
        self.last_rgb_stamp=stamp(self.rgb);self.last_cloud_stamp=stamp(self.cloud)
        try:
            rgb=self.bridge.imgmsg_to_cv2(self.rgb,desired_encoding='rgb8')
            if self.depth.encoding!='32FC1':raise ValueError('Expected depth in metres as 32FC1.')
            depth=self.bridge.imgmsg_to_cv2(self.depth,desired_encoding='32FC1')
            if self.info.width!=rgb.shape[1] or self.info.height!=rgb.shape[0]:raise ValueError('CameraInfo dimensions mismatch.')
            intrinsic=[self.info.k[i] for i in [0,4,2,5]]
            if min(intrinsic[:2])<=0:raise ValueError('Camera focal lengths must be positive.')
            if self.cloud.header.frame_id!='factory_lidar':raise ValueError('Unexpected LiDAR frame: '+self.cloud.header.frame_id)
            raw=point_cloud2.read_points_numpy(self.cloud,field_names=('x','y','z'),skip_nans=True).reshape(-1,3)
            r,t=sensor_transform(self.c['lidar'])
            candidates,mask=detect(rgb,depth,intrinsic,raw@r.T+t,self.c)
            stable=self.stability.update(candidates)
            self.publish([stable] if stable else [],'stable' if stable else 'uncertain_or_not_stationary',stamp(self.rgb))
            debug=self.bridge.cv2_to_imgmsg(mask,encoding='mono8');debug.header=self.rgb.header;self.debug.publish(debug)
            clear=Marker(action=Marker.DELETEALL)
            markers=[clear]
            if stable:
                marker=Marker();marker.header.frame_id='world';marker.ns='detected';marker.id=1
                marker.type=Marker.TEXT_VIEW_FACING;marker.action=Marker.ADD
                marker.pose.position.x,marker.pose.position.y=stable['xyz'][:2]
                marker.pose.position.z=stable['top_z']+0.04;marker.pose.orientation.w=1.0
                marker.scale.z=0.02;marker.color.r=marker.color.g=marker.color.b=marker.color.a=1.0
                marker.text=f"{stable['shape']} | lidar {stable['lidar_points']} points";markers.append(marker)
            self.markers.publish(MarkerArray(markers=markers))
        except Exception as error:
            self.stability.update([]);self.publish([],'sensor_processing_error',now)
            if time.monotonic()-self.last_warning>3:
                self.get_logger().error(str(error));self.last_warning=time.monotonic()


def main():
    rclpy.init();node=Perception()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
