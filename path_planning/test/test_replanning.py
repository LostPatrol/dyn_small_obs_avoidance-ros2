"""Real DDS checks for ROS1 goal/cloud event planning and persistent failed replans."""
import math
import os
import signal
import subprocess
import time

import pytest

# Keep this process and its child isolated from flight hardware and other DDS tests.
os.environ['ROS_DOMAIN_ID'] = '231'
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
from std_msgs.msg import Header, Int64


class PlannerProbe:
    """Publish one acknowledged observation at a time, and collect all planning outcomes."""
    def __init__(self, proc, log_path):
        self.proc, self.log_path = proc, log_path
        self.node = rclpy.create_node('planner_replanning_test')
        self.clouds = self.node.create_publisher(PointCloud2, 'cloud', qos_profile_sensor_data)
        self.odoms = self.node.create_publisher(Odometry, 'odom', qos_profile_sensor_data)
        self.goals = self.node.create_publisher(PoseStamped, 'goal', 1)
        self.indices = self.node.create_publisher(Int64, '/demo_node/trajectory_time_index', 10)
        self.results, self.paths, self.filtered = [], [], []
        self.node.create_subscription(PlanResult, 'plan_result', self.results.append, 100)
        self.node.create_subscription(Path, 'kino_path', self.paths.append, 100)
        filtered_sub = self.node.create_subscription(
            PointCloud2, 'filtered_cloud', self.filtered.append, qos_profile_sensor_data)
        self.wait(lambda: all(p.get_subscription_count() for p in
                              (self.clouds, self.odoms, self.goals, self.indices)) and
                  filtered_sub.get_publisher_count() > 0)
        # The filtered echo is best effort; allow endpoint matching to finish before its first frame.
        self.spin(.3)

    def spin(self, seconds=.08):
        """Drain DDS callbacks for a bounded interval and catch child crashes immediately."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert self.proc.poll() is None, self.log_path.read_text()
            rclpy.spin_once(self.node, timeout_sec=.01)

    def wait(self, predicate, seconds=5):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.spin(.02)
            if predicate():
                return
        assert predicate(), ([(r.status, r.detail, r.sequence) for r in self.results[-10:]],
                             self.log_path.read_text())

    def observe(self, points=((-20., -20., 1.),), position=(0., 0., 1.)):
        """Refresh actual state before the cloud event; its echo proves map ingestion occurred."""
        odom = Odometry()
        odom.header = Header(frame_id='map', stamp=self.node.get_clock().now().to_msg())
        odom.child_frame_id = 'body'
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = position
        odom.pose.pose.orientation.w = 1.
        self.odoms.publish(odom)
        self.spin(.03)
        header = Header(frame_id='map', stamp=self.node.get_clock().now().to_msg())
        self.clouds.publish(create_cloud_xyz32(header, points))
        self.wait(lambda: any(c.header.stamp == header.stamp for c in self.filtered))
        self.spin()

    def goal(self, x=4., y=0.):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = x, y, 1.
        goal.pose.orientation.w = 1.
        self.goals.publish(goal)

    def index(self, value):
        """Wait for delivery before the following goal/cloud event exercises index throttling."""
        self.indices.publish(Int64(data=value))
        self.spin()

    def outcome(self, first, status):
        self.wait(lambda: any(r.status == status for r in self.results[first:]))
        return next(r for r in self.results[first:] if r.status == status)

    def quiet(self, observations=5, position=(0., 0., 1.), points=((-20., -20., 1.),)):
        """Neither fresh safe clouds nor the 10 Hz timer may emit another attempt or Path."""
        self.spin()
        before = len(self.results), len(self.paths)
        for _ in range(observations):
            self.observe(points=points, position=position)
        self.spin(.35)  # Includes several diagnostic timer periods while inputs remain fresh.
        assert (len(self.results), len(self.paths)) == before, [
            (r.status, r.detail, r.planning_ms) for r in self.results[before[0]:]]


@pytest.fixture
def planner(tmp_path):
    """Run the installed executable, with two short-lived map banks for deterministic recovery."""
    exe = get_package_prefix('path_planning') + '/lib/path_planning/path_planning_node'
    log_path = tmp_path / 'replanning.log'
    with log_path.open('w') as log:
        proc = subprocess.Popen([
            exe, '--ros-args', '-p', 'planning_frame:=map', '-p', 'input_timeout:=2.0',
            '-p', 'stamp_clock:=receive', '-p', 'cloud.blind_radius:=0.0',
            '-p', 'search.tree_period:=1', '-p', 'search.safe_distance:=0.25',
            '-p', 'search.voxel_size:=0.1', '-p', 'sample_step:=0.02',
            '-r', 'time_index:=/demo_node/trajectory_time_index',
        ], stdout=log, stderr=log)
        rclpy.init()
        probe = None
        try:
            probe = PlannerProbe(proc, log_path)
            yield probe
        finally:
            if probe:
                probe.node.destroy_node()
            rclpy.shutdown()
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5)
    assert proc.returncode in (0, -signal.SIGINT), log_path.read_text()
    assert 'AddressSanitizer' not in log_path.read_text()
    assert 'runtime error:' not in log_path.read_text()


def legacy_samples(result):
    """Reconstruct the wrapper's 0.01 s cache from exact coefficients, independently of output step."""
    samples = []
    for segment in result.segments:
        count = max(1, math.ceil(segment.duration/.01))
        for i in range(0 if not samples else 1, count + 1):
            t = segment.duration*i/count
            samples.append(tuple(c[0] + c[1]*t + c[2]*t*t + c[3]*t*t*t
                                 for c in (segment.x, segment.y, segment.z)))
    return samples


def test_goal_cloud_events_and_failed_replan_recovery(planner):
    """A safe path stays stable; unsafe history latches replanning until a successful recovery."""
    p = planner
    p.observe()
    first = len(p.results)
    p.goal()
    initial = p.outcome(first, PlanResult.REACH_END)
    p.spin()
    assert sum(r.status == PlanResult.REACH_END for r in p.results[first:]) == 1
    assert len(p.paths) == 1
    assert initial.trajectory.points and initial.segments
    p.quiet()

    # A second received goal at the same coordinates is still an explicit planning event.
    first = len(p.results)
    p.goal()
    repeated = p.outcome(first, PlanResult.REACH_END)
    assert repeated.goal_revision == initial.goal_revision + 1
    p.quiet()

    # Place a centroid on an actually checked legacy cache sample near x=2, away from start/goal.
    checked = legacy_samples(repeated)[::40]
    blocker = min(checked, key=lambda xyz: abs(xyz[0]-2.))
    assert 1. < blocker[0] < 3.
    first = len(p.results)
    p.observe(points=((-20., -20., 1.), blocker))
    detour = p.outcome(first, PlanResult.REACH_END)
    assert detour.goal_revision == repeated.goal_revision
    assert detour.sequence > repeated.sequence
    assert any(abs(t.transforms[0].translation.y) > .1 or
               abs(t.transforms[0].translation.z-1.) > .1 for t in detour.trajectory.points)
    p.quiet()

    # Blocking the goal and a checked old-path point makes replanning fail; no failed output
    # may overwrite the last successful preview, and clearing both banks must retry the latch.
    samples = legacy_samples(detour)[::40]
    blocker = min(samples, key=lambda xyz: abs(xyz[0]-2.))
    first, path_count = len(p.results), len(p.paths)
    p.observe(points=((-20., -20., 1.), blocker, (4., 0., 1.)))
    failed = p.outcome(first, PlanResult.NO_PATH)
    assert not failed.segments and not failed.trajectory.points
    assert len(p.paths) == path_count
    first = len(p.results)
    p.observe()  # One old bank still contains the blocked goal.
    still_failed = p.outcome(first, PlanResult.NO_PATH)
    assert still_failed.sequence > failed.sequence
    assert len(p.paths) == path_count
    first = len(p.results)
    p.observe()  # Both banks are now clear; no extra goal event is sent.
    recovered = p.outcome(first, PlanResult.REACH_END)
    assert recovered.goal_revision == failed.goal_revision
    p.quiet()

    # Actual odometry within 0.3 m of goal suppresses even an unsafe old-path cloud event.
    p.quiet(position=(3.8, 0., 1.), points=((-20., -20., 1.), (2., 0., 1.), (4., 0., 1.)))


def test_initial_failure_needs_another_goal_event(planner):
    """Failure without an old successful path cannot turn safe cloud/timer updates into searches."""
    p = planner
    p.observe(points=((-20., -20., 1.), (4., 0., 1.)))
    first = len(p.results)
    p.goal()
    failed = p.outcome(first, PlanResult.NO_PATH)
    assert not failed.segments and not p.paths
    p.quiet()  # Clears both banks while proving there is no automatic first-plan retry.
    first = len(p.results)
    p.goal()
    recovered = p.outcome(first, PlanResult.REACH_END)
    assert recovered.goal_revision == failed.goal_revision + 1
    p.quiet()


def test_external_index_throttle_and_invalid_indices(planner):
    """Only a valid executor-index advance of at least ten samples permits another attempt."""
    p = planner
    p.observe()
    first = len(p.results)
    p.goal()
    initial = p.outcome(first, PlanResult.REACH_END)
    p.spin()

    p.index(0)  # Previous index is -1, so the absolute difference is below ten.
    first, path_count = len(p.results), len(p.paths)
    p.goal()
    p.quiet(observations=2)
    assert len(p.results) == first and len(p.paths) == path_count

    p.index(10)
    p.goal()
    advanced = p.outcome(first, PlanResult.REACH_END)
    assert advanced.goal_revision == initial.goal_revision + 2
    # The legacy index+3 position comes from the 0.01 s cache, although public output is 0.02 s.
    samples = legacy_samples(initial)
    expected = samples[13]
    actual = advanced.trajectory.points[0].transforms[0].translation
    assert (actual.x, actual.y, actual.z) == pytest.approx(expected, abs=1e-9)
    expected_velocity = tuple((after-before)/.04 for before, after in zip(samples[11], samples[15]))
    velocity = advanced.trajectory.points[0].velocities[0].linear
    assert (velocity.x, velocity.y, velocity.z) == pytest.approx(expected_velocity, abs=1e-9)
    p.spin()

    first, path_count = len(p.results), len(p.paths)
    for index in (15, -2, 1000000, 2**63-1):
        p.index(index)
        p.goal()
        p.quiet(observations=1)
        assert len(p.results) == first and len(p.paths) == path_count
