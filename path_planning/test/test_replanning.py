"""Real DDS checks for event, elapsed-time and displacement replanning without flight hardware."""
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
        # Later stale-input diagnostics must not hide the event's original search failure.
        assert predicate(), {
            'search_attempts': [(r.sequence, r.goal_revision, r.status, r.detail, r.planning_ms,
                                 r.map_points) for r in self.results if r.planning_ms > 0],
            'recent_results': [(r.sequence, r.goal_revision, r.status, r.detail, r.planning_ms)
                               for r in self.results[-10:]],
            'node_log': self.log_path.read_text(),
        }

    def odometry(self, position=(0., 0., 1.)):
        """Publish actual state without a cloud event to isolate displacement triggers."""
        odom = Odometry()
        odom.header = Header(frame_id='map', stamp=self.node.get_clock().now().to_msg())
        odom.child_frame_id = 'body'
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = position
        odom.pose.pose.orientation.w = 1.
        self.odoms.publish(odom)
        self.spin(.03)

    def observe(self, points=((-20., -20., 1.),), position=(0., 0., 1.)):
        """Refresh actual state before the cloud event; its echo proves map ingestion occurred."""
        self.odometry(position)
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
        assert len(self.paths) == before[1]
        assert all(not r.target_id and r.status == r.WAITING_FOR_INPUT and r.planning_ms == 0
                   for r in self.results[before[0]:]), [
            (r.status, r.detail, r.planning_ms) for r in self.results[before[0]:]]


@pytest.fixture
def planner(tmp_path, request):
    """Run the installed executable, with two short-lived map banks for deterministic recovery."""
    exe = get_package_prefix('path_planning') + '/lib/path_planning/path_planning_node'
    log_path = tmp_path / 'replanning.log'
    # Legacy event tests isolate their original triggers; new tests select their own thresholds.
    parameters = getattr(request, 'param', {'replan_interval': 60., 'replan_distance': 100.}) or {}
    arguments = [arg for name, value in parameters.items() for arg in ('-p', f'{name}:={value}')]
    with log_path.open('w') as log:
        proc = subprocess.Popen([
            exe, '--ros-args', '-p', 'planning_frame:=map', '-p', 'input_timeout:=2.0',
            '-p', 'stamp_clock:=receive', '-p', 'cloud.blind_radius:=0.0',
            '-p', 'search.tree_period:=1', '-p', 'search.safe_distance:=0.25',
            # Explicit legacy test envelope keeps event/collision scenes independent of default tuning.
            '-p', 'search.search_budget:=0.3',
            '-p', 'search.max_vel:=2.0', '-p', 'search.max_acc:=2.0',
            '-p', 'search.max_vel_xy:=2.82842712474619',
            '-p', 'search.max_acc_xy:=2.82842712474619',
            '-p', 'search.voxel_size:=0.1', '-p', 'sample_step:=0.02',
            '-r', 'time_index:=/demo_node/trajectory_time_index',
        ] + arguments, stdout=log, stderr=log)
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


def test_new_goal_clears_execution_index_and_old_path(planner):
    """A new target cannot use the preceding target's index or cached path as its start."""
    p = planner
    p.observe()
    first = len(p.results)
    p.goal()
    initial = p.outcome(first, PlanResult.REACH_END)
    for index in (0, 10, -2, 1000000, 2**63-1):
        p.index(index)
        first = len(p.results)
        p.goal(x=3.5)
        changed = p.outcome(first, PlanResult.REACH_END)
        assert changed.goal_revision > initial.goal_revision
        assert changed.target_id != initial.target_id
        actual = changed.trajectory.points[0].transforms[0].translation
        assert (actual.x, actual.y, actual.z) == pytest.approx((0., 0., 1.))
        p.quiet(observations=1)


@pytest.mark.parametrize('planner', [{'replan_interval': '.inf',
                                     'replan_distance': '.inf'}], indirect=True)
def test_active_far_goal_heartbeat_with_disabled_forced_replanning(planner):
    """Infinite thresholds preserve active identity heartbeats and obstacle-triggered searches."""
    p = planner
    p.observe()
    first = len(p.results)
    p.goal()
    initial = p.outcome(first, PlanResult.REACH_END)
    p.wait(lambda: len(p.paths) == 1)
    deadline = time.monotonic() + 3.3
    while time.monotonic() < deadline:
        # Movement beyond the default distance trigger must also leave the safe curve intact.
        p.observe(position=(1.2, 0., 1.))
    searches = [r for r in p.results[first:] if r.target_id]
    heartbeats = [r for r in p.results[first:] if not r.target_id]
    assert searches == [initial]
    assert len(p.paths) == 1 and len(heartbeats) >= 3
    assert all(r.status == PlanResult.WAITING_FOR_INPUT and r.planning_ms == 0
               and not r.segments and not r.trajectory.points
               and r.planner_session == initial.planner_session
               and r.goal_revision == initial.goal_revision for r in heartbeats)
    assert all(a.sequence < b.sequence for a, b in zip(heartbeats, heartbeats[1:]))

    blocker = min(legacy_samples(initial)[::40], key=lambda xyz: abs(xyz[0]-2.))
    first = len(p.results)
    p.observe(points=((-20., -20., 1.), blocker), position=(1.2, 0., 1.))
    detour = p.outcome(first, PlanResult.REACH_END)
    assert detour.target_id == initial.target_id and detour.goal_revision == initial.goal_revision
    assert detour.trajectory.points[0].transforms[0].translation.x == pytest.approx(1.2)
    assert any(abs(t.transforms[0].translation.y) > .1 or
               abs(t.transforms[0].translation.z-1.) > .1 for t in detour.trajectory.points)


@pytest.mark.parametrize('planner', [None], indirect=True)
def test_default_interval_refreshes_without_cloud_or_index_advance(planner):
    """Default 0.2 s refresh repeats a safe plan even with a stationary execution index."""
    p = planner
    p.observe()
    first = len(p.results)
    p.goal()
    initial = p.outcome(first, PlanResult.REACH_END)
    first = len(p.results)
    p.index(0)
    refreshed = p.outcome(first, PlanResult.REACH_END)
    elapsed = (refreshed.header.stamp.sec-initial.header.stamp.sec +
               (refreshed.header.stamp.nanosec-initial.header.stamp.nanosec)*1e-9)
    assert .19 <= elapsed < .6
    assert refreshed.goal_revision == initial.goal_revision
    # Result and Path travel on separate DDS topics; delivery order is not guaranteed.
    p.wait(lambda: len(p.paths) >= 2)


@pytest.mark.parametrize('planner', [{'replan_interval': 60.}], indirect=True)
def test_default_distance_uses_last_attempt_and_actual_odometry(planner):
    """Default 1 m trigger includes equality and resets its baseline after each attempt."""
    p = planner
    p.observe()
    first = len(p.results)
    p.goal()
    p.outcome(first, PlanResult.REACH_END)
    p.index(0)
    first = len(p.results)
    p.odometry((.75, 0., 1.))
    p.spin(.15)
    assert len(p.results) == first
    p.odometry((1., 0., 1.))
    moved = p.outcome(first, PlanResult.REACH_END)
    start = moved.trajectory.points[0].transforms[0].translation
    assert (start.x, start.y, start.z) == pytest.approx((1., 0., 1.))
    first = len(p.results)
    p.odometry((1.75, 0., 1.))
    p.spin(.15)
    assert len(p.results) == first
    p.odometry((2., 0., 1.))
    moved = p.outcome(first, PlanResult.REACH_END)
    assert moved.trajectory.points[0].transforms[0].translation.x == pytest.approx(2.)


@pytest.mark.parametrize('planner', [{'replan_interval': 1.5, 'replan_distance': 100.}], indirect=True)
def test_elapsed_refresh_shortens_path_after_obstacle_disappears(planner):
    """Once both map banks clear, time refresh replaces a still-safe detour with a straight path."""
    p = planner
    p.observe()
    first = len(p.results)
    p.goal()
    initial = p.outcome(first, PlanResult.REACH_END)
    blocker = min(legacy_samples(initial)[::40], key=lambda xyz: abs(xyz[0]-2.))
    first = len(p.results)
    p.observe(points=((-20., -20., 1.), blocker))
    detour = p.outcome(first, PlanResult.REACH_END)
    assert any(abs(t.transforms[0].translation.y) > .1 or
               abs(t.transforms[0].translation.z-1.) > .1 for t in detour.trajectory.points)
    first = len(p.results)
    p.observe()
    p.observe()
    assert len(p.results) == first
    refreshed = p.outcome(first, PlanResult.REACH_END)
    assert refreshed.goal_revision == detour.goal_revision
    assert all(abs(t.transforms[0].translation.y) < 1e-6 and
               abs(t.transforms[0].translation.z-1.) < 1e-6 for t in refreshed.trajectory.points)

@pytest.mark.parametrize('planner', [{'replan_interval': .2,
                                     'search.vertical_distance_weight': 10.,
                                     'search.max_vel': 1., 'search.max_acc': 2.,
                                     'search.max_vel_xy': 1., 'search.max_vel_z': .2,
                                     'search.max_acc_xy': 2., 'search.max_acc_z': 2.}], indirect=True)
def test_set_goal_identity_idempotence_cancel_and_native_limits(planner):
    """Native lifecycle shares map/timer planning and carries exact target identities."""
    from path_planning.srv import SetGoal, PlanMotion
    p = planner
    client = p.node.create_client(SetGoal, 'set_goal')
    motion = p.node.create_client(PlanMotion, 'plan_motion')
    assert client.wait_for_service(timeout_sec=5)
    assert motion.wait_for_service(timeout_sec=5)

    def call(target_id='bridge-a', active=True, goal=(3., .5, 2.4)):
        """Send one lifecycle request and return the authoritative acknowledgement."""
        request = SetGoal.Request()
        request.header.frame_id = 'map'
        request.target_id, request.active = target_id, active
        request.goal.x, request.goal.y, request.goal.z = goal
        future = client.call_async(request)
        p.wait(future.done)
        return future.result()

    p.observe()
    p.wait(lambda: bool(p.results))  # Wait for the one-second idle identity heartbeat.
    assert all(not r.target_id and r.planning_ms == 0 for r in p.results)  # Identity only while idle.
    assert p.results and all((r.max_velocity_xy, r.max_velocity_z,
                             r.max_acceleration_xy, r.max_acceleration_z) == (1., .2, 2., 2.)
                            for r in p.results)
    accepted = call()
    assert accepted.accepted and accepted.planner_session
    planned = p.outcome(0, PlanResult.REACH_END)
    assert planned.target_id == 'bridge-a'
    assert planned.planner_session == accepted.planner_session
    assert planned.goal_revision == accepted.goal_revision
    for point in planned.trajectory.points:
        v, a = point.velocities[0].linear, point.accelerations[0].linear
        assert math.hypot(v.x, v.y) <= 1.+1e-8 and abs(v.z) <= .2+1e-8
        assert math.hypot(a.x, a.y) <= 2.+1e-8 and abs(a.z) <= 2.+1e-8
    assert (planned.max_velocity_xy, planned.max_velocity_z,
            planned.max_acceleration_xy, planned.max_acceleration_z) == (1., .2, 2., 2.)
    duration = sum(s.duration for s in planned.segments)
    final_time = planned.trajectory.points[-1].time_from_start
    assert abs(final_time.sec + final_time.nanosec*1e-9-duration) < 1e-8
    assert any(s.z[2] != 0 or s.z[3] != 0 for s in planned.segments)  # Native Z acceleration.
    unchanged = call()
    assert unchanged.accepted and unchanged.goal_revision == accepted.goal_revision
    refused = call(target_id='bridge-other', active=False)
    assert not refused.accepted and refused.goal_revision == accepted.goal_revision

    # Geometry inspection keeps the autonomous target identity/revision intact.
    request = PlanMotion.Request()
    request.header = Header(frame_id='map', stamp=p.node.get_clock().now().to_msg())
    request.mode = PlanMotion.Request.CHECK_LINE
    request.start.z = request.goal.z = 1.
    request.goal.x = .5
    future = motion.call_async(request)
    p.wait(future.done)
    result = future.result().result
    assert result.status == PlanResult.REACH_END and result.planner_session == accepted.planner_session
    assert not result.target_id
    assert (result.max_velocity_xy, result.max_velocity_z,
            result.max_acceleration_xy, result.max_acceleration_z) == (1., .2, 2., 2.)
    still_same = call()
    assert still_same.goal_revision == accepted.goal_revision

    cancelled = call(active=False)
    assert cancelled.accepted and cancelled.goal_revision == accepted.goal_revision+1
    p.spin(.1)
    before = len(p.results)
    p.observe(); p.spin(.5)
    assert all(not r.target_id and r.planning_ms == 0 for r in p.results[before:])
    assert call(active=False).goal_revision == cancelled.goal_revision
    next_goal = call(target_id='bridge-b', goal=(.5, 0., 1.))
    assert next_goal.accepted and next_goal.goal_revision == cancelled.goal_revision+1
    changed = p.outcome(before, PlanResult.REACH_END)
    assert changed.target_id == 'bridge-b' and changed.map_points > 0


@pytest.mark.parametrize('planner', [{'replan_interval': 60., 'replan_distance': 100.}], indirect=True)
def test_near_goal_fault_recovery_without_resetting_goal(planner):
    """A short input fault cannot leave a near-goal target permanently without an executable curve."""
    from path_planning.srv import SetGoal
    p = planner
    client = p.node.create_client(SetGoal, 'set_goal')
    assert client.wait_for_service(timeout_sec=5)
    p.observe()
    request = SetGoal.Request()
    request.header.frame_id = 'map'; request.target_id = 'near-goal'; request.active = True
    request.goal.x = .2; request.goal.z = 1.
    future = client.call_async(request); p.wait(future.done)
    accepted = future.result()
    initial = p.outcome(0, PlanResult.REACH_END)
    assert initial.goal_revision == accepted.goal_revision
    first = len(p.results)
    p.spin(1.1)
    heartbeats = [r for r in p.results[first:] if not r.target_id]
    assert heartbeats and all(r.status == r.WAITING_FOR_INPUT and r.planning_ms == 0
                              and not r.segments for r in heartbeats)
    # A bad cloud frame revokes the successful curve through an ordinary diagnostic result.
    header = Header(frame_id='wrong', stamp=p.node.get_clock().now().to_msg())
    first = len(p.results)
    p.clouds.publish(create_cloud_xyz32(header, [(-20., -20., 1.)]))
    failed = p.outcome(first, PlanResult.FRAME_MISMATCH)
    assert failed.target_id == request.target_id and not failed.segments
    first = len(p.results)
    p.observe(position=(.02, 0., 1.))
    recovered = p.outcome(first, PlanResult.REACH_END)
    assert recovered.goal_revision == accepted.goal_revision
    assert recovered.trajectory.points[0].transforms[0].translation.x == pytest.approx(.02)
    first = len(p.results)
    for _ in range(3):
        future = client.call_async(request); p.wait(future.done)
        assert future.result().goal_revision == accepted.goal_revision
    p.spin(.3)
    assert not any(r.target_id and r.planning_ms > 0 for r in p.results[first:])


def test_idle_heartbeat_exposes_native_restart_session(planner):
    """A restarted idle native node exposes its new session without searching or a fresh goal."""
    p = planner
    p.wait(lambda: any(not r.target_id and r.status == r.WAITING_FOR_INPUT for r in p.results))
    old = p.results[-1]
    assert old.planner_session and old.planning_ms == 0 and not old.segments
    p.spin(1.1)
    newer = p.results[-1]
    assert newer.sequence > old.sequence and newer.planner_session == old.planner_session
    first = len(p.results)
    original = p.proc
    original.send_signal(signal.SIGINT); original.wait(timeout=5)
    restart_log = p.log_path.with_name('restarted.log')
    with restart_log.open('w') as log:
        restarted = subprocess.Popen(original.args, stdout=log, stderr=log)
        p.proc = restarted
        try:
            p.wait(lambda: any(r.planner_session != old.planner_session for r in p.results[first:]))
            heartbeat = next(r for r in p.results[first:] if r.planner_session != old.planner_session)
            assert heartbeat.status == PlanResult.WAITING_FOR_INPUT and not heartbeat.target_id
            assert heartbeat.planning_ms == 0 and not heartbeat.segments and not heartbeat.trajectory.points
            assert heartbeat.header.stamp.sec >= old.header.stamp.sec
        finally:
            restarted.send_signal(signal.SIGINT); restarted.wait(timeout=5)
            p.proc = original
    assert restarted.returncode in (0, -signal.SIGINT), restart_log.read_text()
