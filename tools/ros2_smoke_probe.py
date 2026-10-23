"""Verify ROS image, camera calibration and pose delivery; optional no-fire gimbal command."""
import argparse
import json
import math
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--send-test-command', action='store_true', help='Send yaw=-5 then +5 degrees, distance=4m, fire=False. Use with an isolated simulator only.')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('rm_sim_smoke_probe')
    images, infos, poses = [], [], []
    quaternions = []
    pose_times = []
    sent = 0
    node.create_subscription(CompressedImage, '/image_compressed', lambda m: images.append((len(m.data), m.format)), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/camera_info', lambda m: infos.append((m.width, m.height, list(m.k))), qos_profile_sensor_data)
    def pose(message):
        q = message.pose.orientation
        quaternions.append([q.x, q.y, q.z, q.w])
        pose_times.append(time.monotonic())
        poses.append(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
    node.create_subscription(PoseStamped, '/gimbal_pose', pose, qos_profile_sensor_data)
    publisher = None
    if args.send_test_command:
        from rm_interfaces.msg import GimbalCmd
        publisher = node.create_publisher(GimbalCmd, '/rune_solver/cmd_gimbal', qos_profile_sensor_data)
    started = time.monotonic()
    while time.monotonic()-started < args.seconds:
        if publisher is not None and pose_times:
            cmd = GimbalCmd()
            cmd.yaw, cmd.pitch = (-5.0 if time.monotonic()-pose_times[0]<4 else 5.0), 0.0
            cmd.yaw_diff, cmd.pitch_diff = 0.0, 0.0
            cmd.distance, cmd.fire_advice = 4.0, False
            publisher.publish(cmd)
            sent += 1
        rclpy.spin_once(node, timeout_sec=0.02)
    delta = max((abs(math.atan2(math.sin(p-poses[0]), math.cos(p-poses[0]))) for p in poses), default=0)
    result = {'images':len(images), 'camera_info':len(infos), 'gimbal_pose':len(poses),
        'last_image_bytes':images[-1][0] if images else None, 'last_camera_info':infos[-1] if infos else None,
        'max_yaw_change_rad':delta, 'command_test':args.send_test_command,
        'commands_sent':sent, 'matched_command_subscriptions':publisher.get_subscription_count() if publisher else None,
        'first_quaternion':quaternions[0] if quaternions else None, 'last_quaternion':quaternions[-1] if quaternions else None}
    passed = len(images)>5 and len(infos)>5 and len(poses)>5
    if args.send_test_command: passed = passed and delta>0.02
    result['result'] = 'PASS' if passed else 'FAIL'
    print(json.dumps(result), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed else 1

if __name__ == '__main__': raise SystemExit(main())
