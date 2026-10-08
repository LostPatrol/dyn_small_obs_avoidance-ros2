"""Real DDS regressions for strict XYZ layouts, bounded outputs, frames and freshness."""
import math
import os
import signal
import struct
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
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header


def alter_cloud(cloud, variant):
    """Build wire-level adversarial schemas or a legal extended, padded organized cloud."""
    if variant == 'duplicate_bad_first':
        cloud.fields.insert(0, PointField(name='x', offset=1024, datatype=PointField.FLOAT32, count=1))
    elif variant == 'duplicate_bad_last':
        cloud.fields.append(PointField(name='x', offset=1024, datatype=PointField.FLOAT32, count=1))
    elif variant == 'duplicate_valid':
        cloud.fields.append(PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1))
    elif variant == 'offset_outside':
        cloud.fields[0].offset = 1024
    elif variant == 'offset_crosses_point':
        cloud.fields[2].offset = 10
    elif variant == 'overlap':
        cloud.fields[1].offset = 2
    elif variant == 'wrong_type':
        cloud.fields[0].datatype = PointField.FLOAT64
    elif variant == 'count_zero':
        cloud.fields[0].count = 0
    elif variant == 'count_two':
        cloud.fields[0].count = 2
    elif variant == 'empty_data':
        cloud.data = b''
    elif variant == 'short_row':
        cloud.row_step = 8
    elif variant == 'extended_padding':
        # z/intensity/y/x deliberately differ from packed XYZ order. Each point has 4 padding
        # bytes and each organized row 8; data uses the declared offsets, not struct order.
        cloud.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                        for name, offset in [('z', 12), ('intensity', 0), ('y', 8), ('x', 4)]]
        cloud.width, cloud.height, cloud.point_step, cloud.row_step = 2, 2, 20, 48
        point = struct.pack('<ffff', 99., -20., -20., 1.) + b'\x00'*4
        cloud.data = (point*2 + b'\x00'*8)*2
    elif variant is not None:
        raise ValueError(variant)
    return cloud


def assert_clean_exit(proc, log_path):
    """Also catch recoverable UBSan diagnostics and sanitizer failures emitted on shutdown."""
    assert proc.returncode in (0, -signal.SIGINT), log_path.read_text()
    log = log_path.read_text()
    assert 'AddressSanitizer' not in log and 'runtime error:' not in log, log


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
                 cloud_stamp=None, malformed=False, position=(0.0, 0.0, 1.0), delayed_pair=False,
                 variant=None):
            stamp = node.get_clock().now().to_msg()
            if delayed_pair:
                stamp = (node.get_clock().now()+rclpy.duration.Duration(seconds=.2)).to_msg()
            cloud = create_cloud_xyz32(Header(stamp=stamp, frame_id=frame), points)
            if cloud_stamp is not None:
                cloud.header.stamp = cloud_stamp
            if malformed:
                cloud.fields = cloud.fields[:2]  # Missing z must fail before PCL conversion.
            alter_cloud(cloud, variant)
            clouds.publish(cloud)
            if delayed_pair:
                time.sleep(.06)  # Cloud arrives before its matching odometry, within the wait budget.
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
                                    'waiting for matched' in r.detail, skewed)
                assert mismatch.map_points == 0
                # Allow source time to catch up to the deliberately future-stamped frame.
                time.sleep(.35)
                wait_for(lambda r: r.status == r.REACH_END, feed)
                feed(delayed_pair=True)
                delayed = wait_for(lambda r: r.status == r.REACH_END and
                                   r.cloud_stamp.sec*10**9+r.cloud_stamp.nanosec >
                                   node.get_clock().now().nanoseconds)
                assert delayed.segments
                time.sleep(.25)
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
            if not blind_radius:
                # Real DDS sends both duplicate orders through the installed PCL-linked wrapper;
                # every invalid schema resets the map and emits no current executable trajectory.
                recovered = wait_for(lambda r: r.status == r.REACH_END, feed)
                for variant in ('duplicate_bad_first', 'duplicate_bad_last', 'duplicate_valid',
                                'offset_outside', 'offset_crosses_point', 'overlap', 'wrong_type',
                                'count_zero', 'count_two', 'empty_data', 'short_row'):
                    invalid = wait_for(lambda r: r.status == r.INVALID_INPUT and
                                       r.sequence > recovered.sequence,
                                       lambda variant=variant: feed(variant=variant))
                    assert not invalid.segments and not invalid.trajectory.points, variant
                    assert invalid.map_points == 0, variant
                    assert all(p.header.stamp != invalid.header.stamp for p in paths), variant
                    recovered = wait_for(lambda r: r.status == r.REACH_END and
                                         r.sequence > invalid.sequence, feed)
                for nonfinite in (float('nan'), float('inf'), -float('inf')):
                    invalid = wait_for(lambda r: r.status == r.INVALID_INPUT and
                                       r.sequence > recovered.sequence,
                                       lambda: feed(points=((nonfinite, -20., 1.),)))
                    assert not invalid.segments and not invalid.trajectory.points
                    assert invalid.map_points == 0
                    recovered = wait_for(lambda r: r.status == r.REACH_END and
                                         r.sequence > invalid.sequence, feed)
                earliest_extended_stamp = node.get_clock().now().nanoseconds
                extended = wait_for(lambda r: r.status == r.REACH_END and
                                    r.sequence > recovered.sequence and
                                    r.cloud_stamp.sec*10**9+r.cloud_stamp.nanosec >= earliest_extended_stamp,
                                    lambda: feed(variant='extended_padding'))
                assert extended.segments and extended.trajectory.points
                assert abs(extended.trajectory.points[-1].transforms[0].translation.x-4.) < 1e-8
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
            # Failures carry empty trajectories in PlanResult, but never erase the RViz preview.
            assert paths and all(p.poses for p in paths)
            assert paths[-1].header.stamp != changed.header.stamp
        finally:
            node.destroy_node()
            rclpy.shutdown()
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5)
            assert_clean_exit(proc, tmp_path / 'node.log')


@pytest.mark.parametrize('limit,expected_status,detail', [
    ('output.max_samples:=2', PlanResult.INVALID_INPUT, 'samples'),
    ('output.max_duration:=0.01', PlanResult.INVALID_INPUT, 'duration'),
    ('output.build_budget:=0.000000000001', PlanResult.TIMEOUT, 'construction budget'),
])
def test_output_limits_process(tmp_path, limit, expected_status, detail):
    """A normal solvable search must fail empty when configured output limits cannot fit it."""
    exe = get_package_prefix('path_planning') + '/lib/path_planning/path_planning_node'
    log_path = tmp_path / 'node.log'
    with log_path.open('w') as log:
        proc = subprocess.Popen([exe, '--ros-args', '-p', 'planning_frame:=map',
                                 '-p', 'input_timeout:=2.0', '-p', limit], stdout=log, stderr=log)
        rclpy.init()
        node = rclpy.create_node('planner_output_limits_test')
        clouds = node.create_publisher(PointCloud2, 'cloud', qos_profile_sensor_data)
        odoms = node.create_publisher(Odometry, 'odom', qos_profile_sensor_data)
        goals = node.create_publisher(PoseStamped, 'goal', 1)
        results, paths = [], []
        node.create_subscription(PlanResult, 'plan_result', results.append, 10)
        node.create_subscription(Path, 'kino_path', paths.append, 10)
        try:
            deadline = time.monotonic()+8
            while time.monotonic() < deadline:
                assert proc.poll() is None, log_path.read_text()
                stamp = node.get_clock().now().to_msg()
                clouds.publish(create_cloud_xyz32(Header(stamp=stamp, frame_id='map'),
                                                  ((-20., -20., 1.),)))
                odom = Odometry()
                odom.header = Header(stamp=stamp, frame_id='map')
                odom.child_frame_id = 'body'
                odom.pose.pose.position.z = 1.
                odom.pose.pose.orientation.w = 1.
                odoms.publish(odom)
                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.pose.position.x, goal.pose.position.z = 4., 1.
                goal.pose.orientation.w = 1.
                goals.publish(goal)
                rclpy.spin_once(node, timeout_sec=.03)
                if results and results[-1].status == expected_status and detail in results[-1].detail:
                    break
                time.sleep(.02)
            else:
                raise AssertionError([(r.status, r.detail) for r in results[-10:]])
            assert not results[-1].segments and not results[-1].trajectory.points
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=.01)
            assert not paths  # Failure must never publish an empty or partially constructed RViz Path.
        finally:
            node.destroy_node()
            rclpy.shutdown()
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5)
            assert_clean_exit(proc, log_path)
