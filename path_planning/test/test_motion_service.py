"""Real isolated DDS native map/service regressions; no odometry, goal publisher or flight hardware."""
import os
import signal
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '224'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ.pop('ROS_DISCOVERY_SERVER', None)

import rclpy
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from path_planning.msg import PlanResult, PolynomialSegment
from path_planning.srv import PlanMotion
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header


def test_service_dual_tree_and_queries(tmp_path):
    """Exercise installed service identity, old planner outputs and geometry on alternating map banks."""
    exe = get_package_prefix('path_planning') + '/lib/path_planning/path_planning_node'
    log_path = tmp_path / 'service.log'
    with log_path.open('w') as log:
        proc = subprocess.Popen([
            exe, '--ros-args', '-p', 'planning_frame:=map', '-p', 'search.tree_period:=1', '-p', 'input_timeout:=2.0',
            '-p', 'search.safe_distance:=0.25', '-p', 'search.voxel_size:=0.1',
            '-p', 'search.search_budget:=0.3', '-p', 'stamp_clock:=receive',
        ], stdout=log, stderr=log)
        rclpy.init()
        node = rclpy.create_node('motion_service_test')
        publisher = node.create_publisher(PointCloud2, 'cloud', qos_profile_sensor_data)
        client = node.create_client(PlanMotion, 'plan_motion')
        filtered = []
        subscription = node.create_subscription(
            PointCloud2, 'filtered_cloud', lambda cloud: filtered.append(cloud.header.stamp),
            qos_profile_sensor_data)

        def wait(predicate, seconds=4):
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.02)
                if predicate():
                    return
            assert predicate(), log_path.read_text()

        def cloud(points, updates=2):
            """Send accepted scans; two updates retire both old banks with tree_period=1."""
            for _ in range(updates):
                header = Header(frame_id='map', stamp=node.get_clock().now().to_msg())
                publisher.publish(create_cloud_xyz32(header, points))
                wait(lambda: header.stamp in filtered)
            return header.stamp

        def request(mode=PlanMotion.Request.CHECK_LINE):
            q = PlanMotion.Request()
            q.header = Header(frame_id='map', stamp=node.get_clock().now().to_msg())
            q.mode = mode
            q.start.z = q.goal.z = 1.0
            q.goal.x = 2.0
            return q

        def call(q):
            """Every query response exposes the installed native dynamics, including failures."""
            future = client.call_async(q)
            wait(future.done)
            response = future.result()
            result = response.result
            assert (result.max_velocity_xy, result.max_velocity_z,
                    result.max_acceleration_xy, result.max_acceleration_z) == (1., .2, 2., 2.)
            return response

        try:
            assert client.wait_for_service(timeout_sec=5), log_path.read_text()
            wait(lambda: publisher.get_subscription_count() > 0)
            assert call(request()).result.status == PlanResult.NO_MAP
            old_stamp = cloud([(1., 0., 1.)])
            response = call(request())
            assert response.planner_session
            session = response.planner_session
            assert response.result.cloud_stamp == old_stamp
            assert response.result.status == PlanResult.NO_PATH
            assert not response.result.segments

            cloud([(10., 10., 1.)], updates=1)
            assert call(request()).result.status == PlanResult.NO_PATH  # Old bank remains.
            new_stamp = cloud([(10., 10., 1.)], updates=1)
            response = call(request())
            assert response.planner_session == session
            assert response.result.cloud_stamp == new_stamp
            assert response.result.status == PlanResult.REACH_END
            assert not response.result.segments
            planned = call(request(PlanMotion.Request.PLAN))
            assert planned.result.status == PlanResult.REACH_END, planned.result.detail
            assert planned.result.segments and planned.result.trajectory.points
            q = request(PlanMotion.Request.VALIDATE_TRAJECTORY)
            q.segments = planned.result.segments
            assert call(q).result.status == PlanResult.REACH_END
            cloud([(2., 0., 1.)])
            assert call(q).result.status == PlanResult.NO_PATH

            q = request(); q.mode = 99
            assert call(q).result.status == PlanResult.INVALID_INPUT
            q = request(); q.start.x = float('nan')
            assert call(q).result.status == PlanResult.INVALID_INPUT
            q = request(); q.header.frame_id = 'odom'
            assert call(q).result.status == PlanResult.FRAME_MISMATCH
            q = request(); q.header.stamp.sec -= 3
            assert call(q).result.status == PlanResult.STALE_INPUT
            q = request(PlanMotion.Request.VALIDATE_TRAJECTORY)
            q.segments = [PolynomialSegment(duration=0.)]
            assert call(q).result.status == PlanResult.INVALID_INPUT

            # Endpoint-only checks would miss this out-and-back polynomial's interior collision.
            cloud([(1., 0., 1.)])
            q = request(PlanMotion.Request.VALIDATE_TRAJECTORY); q.goal.x = 0.
            q.segments = [PolynomialSegment(duration=1., x=[0., 4., -4., 0.],
                                             y=[0., 0., 0., 0.], z=[1., 0., 0., 0.])]
            assert call(q).result.status == PlanResult.NO_PATH
            q.trajectory_elapsed = .9
            assert call(q).result.status == PlanResult.REACH_END
        finally:
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=5)
    assert proc.returncode in (0, -signal.SIGINT), log_path.read_text()
    assert 'runtime error:' not in log_path.read_text()
