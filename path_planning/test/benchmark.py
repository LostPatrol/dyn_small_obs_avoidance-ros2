"""Isolated ROS2 benchmark using a synthetic pole or an unmodified ROS2 cloud/odometry bag.

Run with the project .venv after sourcing install/setup.bash. No flight topics are published.
"""
import argparse
import collections
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time


def command_output(command, cwd=None):
    """Capture optional provenance without hiding a missing tool or failed command."""
    try:
        result = subprocess.run(command, cwd=cwd, text=True, capture_output=True, timeout=10)
        return {'returncode': result.returncode, 'stdout': result.stdout.strip(),
                'stderr': result.stderr.strip()}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {'error': str(error)}


def provenance(config, executable):
    """Bind results to source, loaded configuration, installed binary and host/build facts."""
    root = Path(__file__).resolve().parents[2]
    cache = {}
    for package in ('path_searching', 'path_planning'):
        path = root / 'build' / package / 'CMakeCache.txt'
        if path.exists():
            cache[package] = {line.split('=', 1)[0]: line.split('=', 1)[1]
                              for line in path.read_text().splitlines()
                              if '=' in line and line.startswith(('CMAKE_BUILD_TYPE:',
                                  'CMAKE_CXX_COMPILER:', 'CMAKE_CXX_FLAGS', 'PCL_DIR:',
                                  'Eigen3_DIR:', 'Python3_EXECUTABLE:'))}
        else:
            cache[package] = {'unavailable': str(path)}
    hardware = {}
    for path in ('/sys/firmware/devicetree/base/model', '/sys/class/dmi/id/product_name'):
        if Path(path).exists():
            hardware[path] = Path(path).read_text().strip('\x00\n')
    cpuinfo = Path('/proc/cpuinfo').read_text()
    hardware['cpu_identifiers'] = sorted({line for line in cpuinfo.splitlines()
                                          if line.startswith(('model name', 'Hardware', 'CPU part'))})
    hardware['logical_cpus'] = os.cpu_count()
    hardware['mem_total_kib'] = int(Path('/proc/meminfo').read_text().splitlines()[0].split()[1])
    git_status = command_output(['git', 'status', '--porcelain'], root)
    # Hash exact binary diff bytes; untracked source files are not part of git diff.
    diff = subprocess.run(['git', 'diff', '--binary', 'HEAD'], cwd=root, capture_output=True, check=True).stdout
    untracked = subprocess.run(['git', 'ls-files', '--others', '--exclude-standard', '-z'],
                               cwd=root, capture_output=True, check=True).stdout
    return {
        'source_root': str(root), 'git_sha': command_output(['git', 'rev-parse', 'HEAD'], root),
        'git_status_porcelain': git_status,
        'git_dirty': bool(git_status['stdout']) if git_status.get('returncode') == 0 else None,
        'git_diff_sha256': hashlib.sha256(diff).hexdigest(),
        'git_diff_hash_definition': 'exact stdout bytes of git diff --binary HEAD',
        'untracked_file_sha256': {os.fsdecode(path): hashlib.sha256((root / os.fsdecode(path)).read_bytes()).hexdigest()
                                 for path in untracked.split(b'\x00') if path and (root / os.fsdecode(path)).is_file()},
        'config_path': str(config), 'config_sha256': hashlib.sha256(config.read_bytes()).hexdigest(),
        'config_text': config.read_text(), 'executable': str(executable),
        'executable_sha256': hashlib.sha256(Path(executable).read_bytes()).hexdigest(),
        'kernel': platform.release(), 'platform': platform.platform(), 'machine': platform.machine(),
        'hardware': hardware, 'python': sys.version, 'python_executable': sys.executable,
        'build_cache': cache, 'compiler': command_output(['c++', '--version']),
        'dependencies': command_output(['dpkg-query', '-W',
            '-f=${Package}\t${Version}\n', 'libpcl-dev', 'libeigen3-dev', 'ros-jazzy-rclcpp',
            'ros-jazzy-rclpy', 'ros-jazzy-rmw-fastrtps-cpp', 'ros-jazzy-rmw-cyclonedds-cpp']),
        'environment': {key: os.environ.get(key) for key in
                        ('ROS_DISTRO', 'RMW_IMPLEMENTATION', 'AMENT_PREFIX_PATH',
                         'ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')},
    }


def percentiles(values):
    """Return P50/P95/P99/max; missing or incomparable clock values remain absent."""
    import numpy as np
    values = [value for value in values if value is not None and math.isfinite(value)]
    return np.percentile(values, [50, 95, 99, 100]).tolist() if values else []


def failure_periods(results, began, ended):
    """Time observed failure runs until success or end, including an open final run."""
    periods = []
    start = None
    for result in results:
        if result['status'] not in (1, 2) and start is None:
            start = max(began, result['received_monotonic_s'])
        elif result['status'] in (1, 2) and start is not None:
            periods.append({'start_s': start-began, 'duration_s': result['received_monotonic_s']-start,
                            'open_at_end': False})
            start = None
    if start is not None:
        periods.append({'start_s': start-began, 'duration_s': max(0, ended-start), 'open_at_end': True})
    return periods


def process_usage(pid):
    """Read planner CPU and RSS; stat fields follow the parenthesized process name."""
    stat = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
    status = Path(f'/proc/{pid}/status').read_text().splitlines()
    return {'cpu_s': (int(stat[11])+int(stat[12])) / os.sysconf('SC_CLK_TCK'),
            'rss_kib': int(next(line.split()[1] for line in status if line.startswith('VmRSS:')))}


def resolved_parameters(node, rclpy, proc):
    """Read the running node's complete parameter list and values through ROS services."""
    from rcl_interfaces.srv import GetParameters, ListParameters
    from rclpy.parameter import parameter_value_to_python
    clients = [node.create_client(ListParameters, '/path_planning/list_parameters'),
               node.create_client(GetParameters, '/path_planning/get_parameters')]
    try:
        for client in clients:
            deadline = time.monotonic()+10
            while not client.wait_for_service(timeout_sec=1):
                if proc.poll() is not None:
                    raise RuntimeError(f'planner exited with code {proc.returncode}; inspect the benchmark .log')
                if time.monotonic() >= deadline:
                    raise RuntimeError('planner parameter service unavailable: '+client.srv_name)
        future = clients[0].call_async(ListParameters.Request(depth=0))
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or future.result() is None:
            raise RuntimeError('ListParameters failed')
        names = sorted(future.result().result.names)
        future = clients[1].call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or future.result() is None:
            raise RuntimeError('GetParameters failed')
        return {name: {'type': value.type, 'value': parameter_value_to_python(value)}
                for name, value in zip(names, future.result().values)}
    finally:
        for client in clients:
            node.destroy_client(client)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--bag', type=Path)
    parser.add_argument('--cloud-topic', default='/cloud_registered')
    parser.add_argument('--odom-topic', default='/Odometry')
    parser.add_argument('--goal-offset', type=float, nargs=3, default=[4, 0, 0])
    parser.add_argument('--config', type=Path, help='same config entry as planner.launch.py; defaults to installed planner.yaml')
    parser.add_argument('--search-budget', type=float, help='explicit search.search_budget override in seconds (e.g. 0.08 or 0.3)')
    parser.add_argument('--lower', type=float, nargs=3, help='explicit search.lower override; otherwise use loaded YAML')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 1:
        parser.error('--seconds must be finite and greater than the one-second measurement warmup')
    if args.search_budget is not None and (not math.isfinite(args.search_budget) or args.search_budget <= 0):
        parser.error('--search-budget must be finite and positive')
    os.environ['ROS_DOMAIN_ID'] = '230'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.pop('ROS_DISCOVERY_SERVER', None)
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import deserialize_message
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from path_planning.msg import PlanResult
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
    from std_msgs.msg import Header

    records = []
    if args.bag:
        import rosbag2_py
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=''), rosbag2_py.ConverterOptions('', ''))
        types = {args.cloud_topic: PointCloud2, args.odom_topic: Odometry}
        while reader.has_next():
            topic, data, stamp = reader.read_next()
            if topic in types:
                records.append((stamp, topic, deserialize_message(data, types[topic])))
        if not records or not any(t == args.cloud_topic for _, t, _ in records) or not any(t == args.odom_topic for _, t, _ in records):
            raise RuntimeError('bag must contain both cloud and odometry')
        frame = next(m.header.frame_id for _, t, m in records if t == args.cloud_topic)
    else:
        frame = 'map'
    exe = get_package_prefix('path_planning') + '/lib/path_planning/path_planning_node'
    config = (args.config or Path(get_package_share_directory('path_planning')) / 'config/planner.yaml').resolve()
    # Only source-frame and clock adaptation is automatic; changes to geometry/budget are explicit.
    command = [exe, '--ros-args', '--params-file', str(config), '-p', 'planning_frame:=' + frame,
               '-p', 'stamp_clock:=' + ('receive' if records else 'ros')]
    if args.search_budget is not None:
        command += ['-p', 'search.search_budget:=' + str(args.search_budget)]
    if args.lower is not None:
        command += ['-p', 'search.lower:=' + json.dumps(args.lower)]
    metadata = provenance(config, exe)
    metadata['planner_command'] = command
    args.output.parent.mkdir(parents=True, exist_ok=True)
    log = args.output.with_suffix('.log').open('w')
    proc = subprocess.Popen(command, stdout=log, stderr=log)
    rclpy.init()
    node = rclpy.create_node('standalone_benchmark')
    cloud_pub = node.create_publisher(PointCloud2, 'cloud', qos_profile_sensor_data)
    odom_pub = node.create_publisher(Odometry, 'odom', qos_profile_sensor_data)
    goal_pub = node.create_publisher(PoseStamped, 'goal', 1)
    outcomes = []
    published = {'cloud': collections.OrderedDict(), 'odom': collections.OrderedDict()}

    def stamp_ns(stamp):
        return stamp.sec * 1000000000 + stamp.nanosec

    def publish_input(kind, publisher, message):
        # Keep the first publication of a stamp: duplicate timestamps do not refresh accepted input.
        history = published[kind]
        history.setdefault(stamp_ns(message.header.stamp), time.monotonic())
        if len(history) > 16384:  # Bounded (~41 s at 400 Hz); missing old stamps yield null ages.
            history.popitem(last=False)
        publisher.publish(message)

    def summarize(result):
        # Do not retain thousands of nested trajectory objects per result: doing so makes
        # Python GC stall this same process's sensor publisher and falsifies load measurements.
        received = time.monotonic()
        now_ns = node.get_clock().now().nanoseconds
        result_age = (now_ns-stamp_ns(result.header.stamp))/1e6
        record = {'received_monotonic_s': received, 'received_ros_ns': now_ns,
                  'header_stamp_ns': stamp_ns(result.header.stamp), 'frame_id': result.header.frame_id,
                  'sequence': result.sequence, 'goal_revision': result.goal_revision,
                  'status': result.status, 'detail': result.detail, 'planning_ms': result.planning_ms,
                  'map_points': result.map_points, 'segments': len(result.segments),
                  'trajectory_points': len(result.trajectory.points), 'result_age_ms': result_age}
        for kind, stamp in (('cloud', result.cloud_stamp), ('odom', result.odometry_stamp)):
            source_ns = stamp_ns(stamp)
            sent = published[kind].get(source_ns)
            record[kind+'_stamp_ns'] = source_ns
            record[kind+'_publish_age_at_result_ms'] = (received-sent)*1000 if sent is not None else None
            # Device timestamps in receive mode have no known offset to the host ROS clock.
            record[kind+'_source_age_at_result_ms'] = (now_ns-source_ns)/1e6 if not records and sent is not None else None
        outcomes.append(record)

    node.create_subscription(PlanResult, 'plan_result', summarize, 100)
    usage = []
    try:
        metadata['resolved_parameters'] = resolved_parameters(node, rclpy, proc)
        metadata['rmw_implementation'] = rclpy.utilities.get_rmw_implementation_identifier()
        metadata['numpy_version'] = np.__version__
        if metadata['resolved_parameters']['use_sim_time']['value']:
            raise RuntimeError('benchmark does not publish /clock; use_sim_time must be false')
        warmup = time.monotonic()+1.5
        while time.monotonic()<warmup:
            rclpy.spin_once(node, timeout_sec=0.05)
        began = time.monotonic()
        initial_usage = process_usage(proc.pid)
        deadline = began + args.seconds
        cursor = 0
        first_stamp = records[0][0] if records else 0
        next_feed = began
        latest_odom = None
        goal_sent = False
        while time.monotonic()<deadline:
            assert proc.poll() is None, 'planner process exited'
            elapsed = time.monotonic()-began
            if records:
                while cursor < len(records) and (records[cursor][0]-first_stamp)/1e9 <= elapsed:
                    _, topic, msg = records[cursor]
                    if topic == args.cloud_topic:
                        publish_input('cloud', cloud_pub, msg)
                    else:
                        publish_input('odom', odom_pub, msg)
                        latest_odom = msg
                    cursor += 1
            elif time.monotonic() >= next_feed:
                stamp = node.get_clock().now().to_msg()
                points = [(-20.0, -20.0, 1.0)] + [(2.0, 0.0, float(z)) for z in np.arange(-10, 10, 0.025)]
                publish_input('cloud', cloud_pub, create_cloud_xyz32(Header(stamp=stamp, frame_id=frame), points))
                latest_odom = Odometry()
                latest_odom.header = Header(stamp=stamp, frame_id=frame)
                latest_odom.child_frame_id = 'body'
                latest_odom.pose.pose.position.z = 1.0
                latest_odom.pose.pose.orientation.w = 1.0
                publish_input('odom', odom_pub, latest_odom)
                next_feed += 0.1
            if latest_odom is not None and not goal_sent:
                goal = PoseStamped()
                goal.header.frame_id = frame
                goal.pose.orientation.w = 1.0
                p = latest_odom.pose.pose.position
                goal.pose.position.x = p.x + args.goal_offset[0]
                goal.pose.position.y = p.y + args.goal_offset[1]
                goal.pose.position.z = p.z + args.goal_offset[2]
                goal_pub.publish(goal)
                goal_sent = True
            rclpy.spin_once(node, timeout_sec=0.005)
            if not usage or elapsed > usage[-1]['elapsed_s']+1:
                usage.append({'elapsed_s': elapsed, **process_usage(proc.pid)})
        ended = time.monotonic()
        final_usage = process_usage(proc.pid)
        for record in outcomes:
            record['elapsed_s'] = record['received_monotonic_s']-began
            record['included_in_statistics'] = record['elapsed_s'] >= 1
        measured = [r for r in outcomes if r['included_in_statistics']]
        intervals = np.diff([r['received_monotonic_s'] for r in measured])*1000
        # Failure periods span the full feed window, unlike timing quantiles' one-second exclusion.
        failures = failure_periods([r for r in outcomes if r['elapsed_s'] >= 0], began, ended)
        stats = {
            'source': str(args.bag) if args.bag else 'synthetic vertical pole; stationary odometry',
            'duration_s': ended-began, 'statistics_exclude_first_s': 1, 'results': len(measured),
            'status_counts': dict(collections.Counter(r['status'] for r in measured)),
            'all_status_counts': dict(collections.Counter(r['status'] for r in outcomes)),
            'missing_result_sequences': sum(max(0, newer['sequence']-older['sequence']-1)
                                            for older, newer in zip(outcomes, outcomes[1:])),
            'search_ms_p50_p95_p99_max': percentiles([r['planning_ms'] for r in measured if r['planning_ms']>0]),
            'result_interval_ms_p50_p95_p99_max': percentiles(intervals),
            'age_ms_p50_p95_p99_max': {key: percentiles([r[key] for r in measured]) for key in
                ('result_age_ms', 'cloud_publish_age_at_result_ms', 'odom_publish_age_at_result_ms',
                 'cloud_source_age_at_result_ms', 'odom_source_age_at_result_ms')},
            'max_map_points': max((r['map_points'] for r in measured),default=0),
            'map_points_p50_p95_p99_max': percentiles([r['map_points'] for r in measured]),
            'process_usage': usage,
            'cpu_core_percent': (final_usage['cpu_s']-initial_usage['cpu_s']) / (ended-began)*100,
            'failure_periods': failures,
            'max_consecutive_failure_s': max((p['duration_s'] for p in failures), default=0),
            'bag_messages_published': cursor,
            'metadata': metadata, 'raw_results': outcomes,
            'measurement_scope': 'Client receipt interval and attempt-start/input-publication to receipt ages; '
                'not sensor acquisition-to-actuator control latency. Receive-mode source ages are unknown. '
                'All scalar result records retained, including warmup/failures; polynomial/trajectory arrays omitted.',
        }
        args.output.write_text(json.dumps(stats, indent=2)+'\n')
        print(json.dumps({k:v for k,v in stats.items() if k not in ('process_usage', 'metadata', 'raw_results')},indent=2))
    except BaseException as error:
        # Keep all observations even when the planner exits or the operator interrupts a run.
        args.output.write_text(json.dumps({'metadata': metadata, 'raw_results': outcomes,
                                         'process_usage': usage, 'statistics_complete': False,
                                         'run_error': type(error).__name__ + ': ' + str(error)}, indent=2)+'\n')
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
        log.close()


if __name__ == '__main__':
    main()
