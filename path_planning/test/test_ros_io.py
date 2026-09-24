"""Real DDS/process regression: planning, velocity frames, failure invalidation and freshness."""
import math
import os
import signal
import subprocess
import time
import pytest

os.environ['ROS_DOMAIN_ID'] = '232'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ.pop('ROS_DISCOVERY_SERVER', None)

import rclpy
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from path_planning.msg import PlanResult
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header


@pytest.mark.parametrize('blind_radius', [0.0, 0.5])
def test_planner_process(tmp_path, blind_radius):
    """Exercise installed node, with no hardware/network-domain interaction."""
    exe = get_package_prefix('path_planning') + '/lib/path_planning/path_planning_node'
    with (tmp_path / 'node.log').open('w') as log:
        proc = subprocess.Popen([exe, '--ros-args', '-p', 'planning_frame:=map',
                                 '-p', 'input_timeout:=0.4',
                                 '-p', f'cloud.blind_radius:={blind_radius}',
                                 '-p', 'stamp_clock:=' + ('receive' if blind_radius else 'ros'),
                                 '-p', 'cloud.blind_origin_offset:=[0.2, 0.0, 0.0]',
                                 '-p', 'cloud.blind_pose_tolerance:=0.1'], stdout=log, stderr=log)
        rclpy.init()
        node = rclpy.create_node('planner_io_test')
        clouds = node.create_publisher(PointCloud2, 'cloud', qos_profile_sensor_data)
        odoms = node.create_publisher(Odometry, 'odom', qos_profile_sensor_data)
        goals = node.create_publisher(PoseStamped, 'goal', 1)
        results, paths = [], []
        node.create_subscription(PlanResult, 'plan_result', results.append, 10)
        node.create_subscription(Path, 'kino_path', paths.append, 10)

        def feed(points=((-20.0, -20.0, 1.0),), frame='map', velocity=(0.0, 0.0, 0.0), yaw=0.0,
                 cloud_stamp=None, malformed=False, position=(0.0, 0.0, 1.0)):
            stamp = node.get_clock().now().to_msg()
            cloud = create_cloud_xyz32(Header(stamp=stamp, frame_id=frame), points)
            if cloud_stamp is not None:
                cloud.header.stamp = cloud_stamp
            if malformed:
                cloud.fields = cloud.fields[:2]  # Missing z must fail before PCL conversion.
            clouds.publish(cloud)
            odom = Odometry()
            odom.header = Header(stamp=stamp, frame_id='map')
            odom.child_frame_id = 'body'
            odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = position
            odom.pose.pose.orientation.z = math.sin(yaw/2)
            odom.pose.pose.orientation.w = math.cos(yaw/2)
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = velocity
            odoms.publish(odom)

        def target(x=4.0, y=0.0):
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x, goal.pose.position.z = x, 1.0
            goal.pose.position.y = y
            goal.pose.orientation.w = 1.0
            goals.publish(goal)

        def wait_for(predicate, publish=None, seconds=5):
            results.clear()
            deadline = time.monotonic()+seconds
            while time.monotonic()<deadline:
                assert proc.poll() is None, (tmp_path / 'node.log').read_text()
                if publish:
                    publish()
                rclpy.spin_once(node, timeout_sec=0.03)
                if results and predicate(results[-1]):
                    return results[-1]
                time.sleep(0.02)
            raise AssertionError([(r.status, r.detail, r.planning_ms) for r in results[-10:]])

        try:
            wait_for(lambda r: r.status == r.WAITING_FOR_INPUT)
            ok = wait_for(lambda r: r.status == r.REACH_END, lambda: (feed(), target()))
            assert ok.segments and ok.trajectory.points
            assert abs(ok.trajectory.points[-1].transforms[0].translation.x-4)<1e-8
            assert ok.trajectory.points[0].time_from_start.sec == 0
            assert ok.planning_ms < 100
            # The blind sphere follows translated odometry, not the world origin. A near-body
            # point blocks disabled filtering but is removed before accumulation when enabled.
            wait_for(lambda r: r.status == r.FRAME_MISMATCH,
                     lambda: feed(frame='wrong', position=(10.,20.,1.), yaw=math.pi/2))
            expected = PlanResult.REACH_END if blind_radius else PlanResult.NO_PATH
            body = wait_for(lambda r: r.status == expected and r.goal_revision>ok.goal_revision and
                            (not blind_radius or abs(r.trajectory.points[-1].transforms[0].translation.x-14)<1e-8),
                            lambda: (feed(points=((-20.,-20.,1.),(10.1,20.,1.),(10.,20.6,1.)),
                                          position=(10.,20.,1.), yaw=math.pi/2), target(14.,20.)))
            # The 0.6m point is inside only when the 0.2m sensor offset rotates with yaw.
            assert body.map_points == (1 if blind_radius else 3)
            wait_for(lambda r: r.status == r.FRAME_MISMATCH, lambda: feed(frame='wrong'))
            wait_for(lambda r: r.status == r.REACH_END, lambda: (feed(), target()))
            if blind_radius:
                # Outside the sphere remains an obstacle; an entirely removed frame is not free space.
                wait_for(lambda r: r.status == r.NO_PATH,
                         lambda: feed(points=((-20.,-20.,1.),(-.55,0.,1.))))
                no_map = wait_for(lambda r: r.status == r.NO_MAP,
                                  lambda: feed(points=((.1,0.,1.),)))
                assert no_map.map_points == 0 and not no_map.segments
                wait_for(lambda r: r.status == r.REACH_END, feed)
                # Fresh callbacks with mismatched acquisition times must not crop using wrong poses.
                def skewed():
                    future = (node.get_clock().now()+rclpy.duration.Duration(seconds=.3)).to_msg()
                    feed(cloud_stamp=future)
                mismatch = wait_for(lambda r: r.status == r.STALE_INPUT and
                                    'time-matched' in r.detail, skewed)
                assert mismatch.map_points == 0
                # Allow source time to catch up to the deliberately future-stamped frame.
                time.sleep(.35)
                wait_for(lambda r: r.status == r.REACH_END, feed)
            # Odometry twist is in the child frame: +x body at yaw=90deg becomes +y world.
            moving = wait_for(lambda r: r.status == r.REACH_END and abs(r.segments[0].y[1]-0.3)<1e-6,
                              lambda: feed(velocity=(0.3, 0.0, 0.1), yaw=math.pi/2))
            assert abs(moving.segments[0].x[1])<1e-6
            assert abs(moving.segments[0].z[1]-0.1)<1e-6
            blocked = wait_for(lambda r: r.status == r.NO_PATH,
                               lambda: feed(points=((-20.0,-20.0,1.0),(4.0,0.0,1.0))))
            assert not blocked.segments and not blocked.trajectory.points
            wrong = wait_for(lambda r: r.status == r.FRAME_MISMATCH, lambda: feed(frame='wrong'))
            assert not wrong.segments
            wait_for(lambda r: r.status == r.REACH_END, feed)
            stale = wait_for(lambda r: r.status == r.STALE_INPUT, seconds=2)
            assert not stale.segments and not stale.trajectory.points
            wait_for(lambda r: r.status == r.REACH_END, feed)
            empty = wait_for(lambda r: r.status == r.INVALID_INPUT, lambda: feed(points=()))
            assert not empty.segments
            wait_for(lambda r: r.status == r.REACH_END, feed)
            malformed = wait_for(lambda r: r.status == r.INVALID_INPUT, lambda: feed(malformed=True))
            assert not malformed.segments
            wait_for(lambda r: r.status == r.REACH_END, feed)
            # A repeated observation cannot extend map freshness while odometry stays fresh.
            frozen = node.get_clock().now().to_msg()
            duplicate = wait_for(lambda r: r.status == r.STALE_INPUT,
                                 lambda: feed(cloud_stamp=frozen), seconds=2)
            assert not duplicate.segments
            wait_for(lambda r: r.status == r.REACH_END, feed)
            negative = node.get_clock().now().to_msg()
            negative.sec = -1
            bad_time = wait_for(lambda r: r.status == r.INVALID_INPUT,
                                lambda: feed(cloud_stamp=negative))
            assert not bad_time.segments
            wait_for(lambda r: r.status == r.REACH_END, feed)
            # A new, blocked goal must carry a new revision and cannot retain the old path.
            changed = wait_for(lambda r: r.status == r.NO_PATH and r.goal_revision > ok.goal_revision,
                               lambda: (feed(points=((-20.0,-20.0,1.0),(2.0,0.0,1.0))), target(2.0)))
            assert not changed.segments
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.01)
            assert paths and not paths[-1].poses
        finally:
            node.destroy_node()
            rclpy.shutdown()
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5)
