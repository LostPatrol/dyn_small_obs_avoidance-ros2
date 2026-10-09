"""Real DDS service contract: actual request state, map replacement and exact suffix validation."""
import os
import signal
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '232'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ.pop('ROS_DISCOVERY_SERVER', None)

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from path_planning.msg import PlanResult, PolynomialSegment
from path_planning.srv import PlanMotion


def test_motion_service_without_odom_or_goal(tmp_path):
    """A service-only process uses the supplied world p/v and fails closed on stale/invalid maps."""
    executable = get_package_prefix('path_planning') + '/lib/path_planning/path_planning_node'
    log_path = tmp_path / 'motion_service.log'
    with log_path.open('w') as log:
        process = subprocess.Popen([executable, '--ros-args', '-p', 'service_only:=true',
                                    '-p', 'planning_frame:=map', '-p', 'cloud.map_mode:=latest_observation',
                                    '-p', 'input_timeout:=1.0', '-p', 'search.search_budget:=0.5',
                                    '-p', 'search.max_horizontal_vel:=1.0',
                                    '-p', 'search.max_vertical_vel:=0.2',
                                    '-p', 'search.max_horizontal_acc:=0.35',
                                    '-p', 'search.max_vertical_acc:=0.15'], stdout=log, stderr=log)
        rclpy.init()
        node = rclpy.create_node('motion_service_test')
        publisher = node.create_publisher(PointCloud2, 'cloud', qos_profile_sensor_data)
        client = node.create_client(PlanMotion, 'plan_motion')
        topic_results = []
        node.create_subscription(PlanResult, 'plan_result', topic_results.append, 10)

        def request(mode=PlanMotion.Request.CHECK_LINE, start=(0., 0., 1.), goal=(4., 0., 1.)):
            message = PlanMotion.Request()
            message.header = Header(stamp=node.get_clock().now().to_msg(), frame_id='map')
            message.mode = mode
            message.start.x, message.start.y, message.start.z = start
            message.goal.x, message.goal.y, message.goal.z = goal
            return message

        def call(message):
            future = client.call_async(message)
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
            assert future.done(), log_path.read_text()
            return future.result()

        def map_result(points, expected, mode=PlanMotion.Request.CHECK_LINE, **kwargs):
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                publisher.publish(create_cloud_xyz32(
                    Header(stamp=node.get_clock().now().to_msg(), frame_id='map'), points))
                rclpy.spin_once(node, timeout_sec=.02)
                response = call(request(mode, **kwargs))
                if response.result.status == expected:
                    return response
            raise AssertionError((response.result.status, response.result.detail, log_path.read_text()))

        try:
            assert client.wait_for_service(timeout_sec=5), log_path.read_text()
            assert call(request()).result.status == PlanResult.NO_MAP
            clear = map_result(((-20., -20., 1.),), PlanResult.REACH_END)
            assert clear.planner_session and not clear.result.segments
            planned = map_result(((-20., -20., 1.),), PlanResult.REACH_END,
                                 PlanMotion.Request.PLAN, start=(1., 1., 1.), goal=(2., 1., 1.))
            assert planned.planner_session == clear.planner_session
            assert abs(planned.result.segments[0].x[0] - 1.) < 1e-9
            assert abs(planned.result.segments[0].y[0] - 1.) < 1e-9
            moving = request(PlanMotion.Request.PLAN, start=(1., 1., 1.), goal=(2., 1., 1.))
            moving.velocity.x = .1
            motion = call(moving)
            assert motion.result.status == PlanResult.REACH_END
            assert abs(motion.result.segments[0].x[1] - .1) < 1e-9
            moving.velocity.x, moving.velocity.y = .8, .8
            moving.header.stamp = node.get_clock().now().to_msg()
            assert call(moving).result.status == PlanResult.INVALID_INPUT
            blocked = map_result(((2., .44, 1.),), PlanResult.NO_PATH)
            assert not blocked.result.segments and not blocked.result.trajectory.points
            map_result(((-20., -20., 1.),), PlanResult.REACH_END)  # explicit replacement clears old map
            map_result(((.5, 0., 1.),), PlanResult.NO_PATH)
            curve = PolynomialSegment(duration=4., x=[0., 1., 0., 0.],
                                      y=[0., 0., 0., 0.], z=[1., 0., 0., 0.])
            validation = request(PlanMotion.Request.VALIDATE_TRAJECTORY)
            validation.segments = [curve]
            assert call(validation).result.status == PlanResult.NO_PATH
            validation.trajectory_elapsed = 2.
            validation.header.stamp = node.get_clock().now().to_msg()
            validated = call(validation)
            assert validated.result.status == PlanResult.REACH_END and not validated.result.segments
            wrong = request(); wrong.header.frame_id = 'wrong'
            assert call(wrong).result.status == PlanResult.FRAME_MISMATCH
            unknown = request(); unknown.mode = 255
            assert call(unknown).result.status == PlanResult.INVALID_INPUT
            stale = request(); stale.header.stamp.sec -= 2
            assert call(stale).result.status == PlanResult.STALE_INPUT
            time.sleep(1.1)
            assert call(request()).result.status == PlanResult.STALE_INPUT
            map_result((), PlanResult.INVALID_INPUT)
            assert not topic_results  # service_only never leaks automatic attempts to standalone topics
            # Process restart must be distinguishable even when sequence counters restart from zero.
            arguments = process.args
            process.send_signal(signal.SIGINT); process.wait(timeout=5)
            process = subprocess.Popen(arguments, stdout=log, stderr=log)
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                if not client.wait_for_service(timeout_sec=.1):
                    continue
                restarted = call(request())
                if restarted.planner_session != clear.planner_session:
                    assert restarted.result.status == PlanResult.NO_MAP
                    break
            else:
                raise AssertionError('planner restart retained its process identity')
        finally:
            node.destroy_node(); rclpy.shutdown()
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5)
            assert process.returncode in (0, -signal.SIGINT), log_path.read_text()
