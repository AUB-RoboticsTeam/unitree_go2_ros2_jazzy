#!/usr/bin/env python3
"""Remove the Go2's own body from the simulated L1 point cloud.

Gazebo Classic's GPU lidar renders the robot's own legs, which the gz-sim lidar
in unitree_go2_ros2_jazzy never shows. The lidar is fixed to `base`, so every
point inside the robot's body box (in the base frame) is dropped.
"""
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
from tf.transformations import quaternion_matrix


class LidarSelfFilter(object):
    def __init__(self):
        self.base_frame = rospy.get_param("~base_frame", "base")
        # Body box in the base frame: trunk, hips and legs above the feet.
        self.box_min = np.array(rospy.get_param("~box_min", [-0.42, -0.22, -0.29]), dtype=np.float32)
        self.box_max = np.array(rospy.get_param("~box_max", [0.40, 0.22, 0.20]), dtype=np.float32)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.rotation = None
        self.translation = None
        self.dtype = None

        self.pub = rospy.Publisher("output", PointCloud2, queue_size=2)
        rospy.Subscriber("input", PointCloud2, self.callback, queue_size=2, buff_size=2 ** 24)

    def lookup_mount(self, sensor_frame):
        # Fixed joint: the mount never changes, so look it up once.
        t = self.tf_buffer.lookup_transform(self.base_frame, sensor_frame, rospy.Time(0),
                                            rospy.Duration(1.0))
        q = t.transform.rotation
        self.rotation = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3].astype(np.float32)
        self.translation = np.array([t.transform.translation.x, t.transform.translation.y,
                                     t.transform.translation.z], dtype=np.float32)

    def callback(self, msg):
        if self.rotation is None:
            try:
                self.lookup_mount(msg.header.frame_id)
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException):
                rospy.logwarn_throttle(5.0, "waiting for %s -> %s transform",
                                       self.base_frame, msg.header.frame_id)
                return
        if self.dtype is None:
            offsets = {f.name: f.offset for f in msg.fields}
            self.dtype = np.dtype({"names": ["x", "y", "z"],
                                   "formats": [np.float32] * 3,
                                   "offsets": [offsets["x"], offsets["y"], offsets["z"]],
                                   "itemsize": msg.point_step})

        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
        points = np.frombuffer(msg.data, dtype=self.dtype)
        xyz = np.stack([points["x"], points["y"], points["z"]], axis=1)
        in_base = xyz @ self.rotation.T + self.translation
        inside = np.all((in_base > self.box_min) & (in_base < self.box_max), axis=1)
        kept = raw[~inside]

        out = PointCloud2()
        out.header = msg.header
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.height = 1
        out.width = kept.shape[0]
        out.row_step = msg.point_step * out.width
        out.is_dense = msg.is_dense
        out.data = kept.tobytes()
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("lidar_self_filter")
    LidarSelfFilter()
    rospy.spin()
