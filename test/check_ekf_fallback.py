"""Synthetic integration check; run ekf.launch.py in an isolated ROS domain first.

Does not open hardware or publish motor commands.
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

rclpy.init()
node = Node('synthetic_fusion_test')
wheel = node.create_publisher(Odometry, '/wheel/odom', 10)
imu = node.create_publisher(Imu, '/zed/zed_node/imu/data', 10)
outputs = []
statuses = []
node.create_subscription(Odometry, '/odom', lambda m: outputs.append(m), 100)
node.create_subscription(String, '/localization/imu_status',
                         lambda m: statuses.append(m.data),
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
tf = StaticTransformBroadcaster(node)
t = TransformStamped()
t.header.stamp = node.get_clock().now().to_msg()
t.header.frame_id = 'base_link'
t.child_frame_id = 'test_imu'
t.transform.rotation.w = 1.0
tf.sendTransform(t)

def phase(use_imu, expected, stale=False, invalid_gyro=False):
    start = time.monotonic()
    begin = len(outputs)
    while time.monotonic() - start < 3.0:
        stamp = node.get_clock().now().to_msg()
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.orientation.w = 1.0
        o.twist.twist.linear.x = 0.2
        o.twist.covariance[0] = 0.01
        o.twist.covariance[35] = 0.02
        wheel.publish(o)
        if use_imu:
            m = Imu()
            m.header.stamp = stamp
            if stale:
                m.header.stamp.sec -= 2
            m.header.frame_id = 'test_imu'
            m.angular_velocity.z = 10.0 if invalid_gyro else 0.2
            m.angular_velocity_covariance[0] = 0.0004
            m.angular_velocity_covariance[4] = 0.0004
            m.angular_velocity_covariance[8] = 0.0004
            imu.publish(m)
        rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(0.02)
    assert len(outputs) - begin > 30, 'EKF stopped publishing'
    assert statuses and statuses[-1] == expected, statuses
    yaw_rate = outputs[-1].twist.twist.angular.z
    assert (yaw_rate > 0.1 if expected == 'wheel_imu' else abs(yaw_rate) < 0.05), yaw_rate
    print(expected, 'stale=', stale, 'yaw_rate=', yaw_rate, flush=True)

try:
    phase(False, 'wheel_only')
    phase(True, 'wheel_imu')
    phase(False, 'wheel_only')
    phase(True, 'wheel_only', stale=True)
    phase(True, 'wheel_only', invalid_gyro=True)
    phase(True, 'wheel_imu')
    for a, b in zip(outputs, outputs[1:]):
        assert abs(b.pose.pose.position.x - a.pose.pose.position.x) < 0.1
        assert abs(b.pose.pose.position.y - a.pose.pose.position.y) < 0.1
    print('PASS: startup without camera, fusion, loss, stale rejection, recovery, continuity')
finally:
    node.destroy_node()
    rclpy.shutdown()
