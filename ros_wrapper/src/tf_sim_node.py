#!/usr/bin/env python3
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import rospy

from ros_conversions import cvt_point2ros_transform


class TFSimNode(object):
    def __init__(self):
        rospy.init_node('tf_sim', anonymous=True)
        self.coords = rospy.get_param('~coords')
        self.base_frame = rospy.get_param('~base_frame', "ump")
        self.map_frame = rospy.get_param('~map_frame', "map")
        self.rate = float(rospy.get_param('~rate', 1.0))

        rospy.Timer(rospy.Duration(1. / self.rate), self.timer_callback)

    def timer_callback(self, event):
        self.publish_tf_transform()

    def publish_tf_transform(self):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame
        t.transform = cvt_point2ros_transform(self.coords)
        br.sendTransform(t)


if __name__ == '__main__':
    try:
        tf_sim_node = TFSimNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
