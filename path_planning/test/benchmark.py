"""Isolated ROS2 benchmark using a synthetic pole or an unmodified ROS2 cloud/odometry bag.

Run with the project .venv after sourcing install/setup.bash. No flight topics are published.
"""
import argparse
import collections
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from types import SimpleNamespace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--bag', type=Path)
    parser.add_argument('--cloud-topic', default='/cloud_registered')
    parser.add_argument('--odom-topic', default='/Odometry')
    parser.add_argument('--goal-offset', type=float, nargs=3, default=[4, 0, 0])
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = '230'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.pop('ROS_DISCOVERY_SERVER', None)
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import deserialize_message
    from ament_index_python.packages import get_package_prefix
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
    args.output.parent.mkdir(parents=True, exist_ok=True)
    log = args.output.with_suffix('.log').open('w')
    proc = subprocess.Popen([exe, '--ros-args', '-p', 'planning_frame:=' + frame,
                             '-p', 'stamp_clock:=' + ('receive' if records else 'ros'),
                             '-p', 'search.lower:=[-50.0,-50.0,-10.0]'], stdout=log, stderr=log)
    rclpy.init()
    node = rclpy.create_node('standalone_benchmark')
    cloud_pub = node.create_publisher(PointCloud2, 'cloud', qos_profile_sensor_data)
    odom_pub = node.create_publisher(Odometry, 'odom', qos_profile_sensor_data)
    goal_pub = node.create_publisher(PoseStamped, 'goal', 1)
    outcomes = []

    def summarize(result):
        # Do not retain thousands of nested trajectory objects per result: doing so makes
        # Python GC stall this same process's sensor publisher and falsifies load measurements.
        outcomes.append((time.monotonic(), SimpleNamespace(
            status=result.status, detail=result.detail, planning_ms=result.planning_ms,
            map_points=result.map_points, segments=len(result.segments))))

    node.create_subscription(PlanResult, 'plan_result', summarize, 100)
    rss = []
    try:
        warmup = time.monotonic()+1.5
        while time.monotonic()<warmup:
            rclpy.spin_once(node, timeout_sec=0.05)
        began = time.monotonic()
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
                        cloud_pub.publish(msg)
                    else:
                        odom_pub.publish(msg)
                        latest_odom = msg
                    cursor += 1
            elif time.monotonic() >= next_feed:
                stamp = node.get_clock().now().to_msg()
                points = [(-20.0, -20.0, 1.0)] + [(2.0, 0.0, float(z)) for z in np.arange(-10, 10, 0.025)]
                cloud_pub.publish(create_cloud_xyz32(Header(stamp=stamp, frame_id=frame), points))
                latest_odom = Odometry()
                latest_odom.header = Header(stamp=stamp, frame_id=frame)
                latest_odom.child_frame_id = 'body'
                latest_odom.pose.pose.position.z = 1.0
                latest_odom.pose.pose.orientation.w = 1.0
                odom_pub.publish(latest_odom)
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
            if len(rss)==0 or elapsed>rss[-1][0]+1:
                text = Path(f'/proc/{proc.pid}/status').read_text()
                rss.append((elapsed, int(next(line.split()[1] for line in text.splitlines() if line.startswith('VmRSS:')))))
        measured = [(t,r) for t,r in outcomes if t >= began+1]
        searches = [r.planning_ms for _,r in measured if r.planning_ms>0]
        intervals = np.diff([t for t,_ in measured])*1000
        cpu = Path(f'/proc/{proc.pid}/stat').read_text().split()
        stats = {
            'source': str(args.bag) if args.bag else 'synthetic vertical pole; stationary odometry',
            'duration_s': args.seconds, 'results': len(measured),
            'status_counts': dict(collections.Counter(r.status for _,r in measured)),
            'search_ms_p50_p95_p99_max': np.percentile(searches,[50,95,99,100]).tolist() if searches else [],
            'result_interval_ms_p50_p95_p99_max': np.percentile(intervals,[50,95,99,100]).tolist() if len(intervals) else [],
            'max_map_points': max((r.map_points for _,r in measured),default=0),
            'rss_kib': rss, 'cpu_core_percent': (int(cpu[13])+int(cpu[14])) / os.sysconf('SC_CLK_TCK') / (time.monotonic()-began+1.5)*100,
            'bag_messages_published': cursor,
            'first_results': [{'status': r.status, 'detail': r.detail, 'segments': r.segments} for _,r in measured[:5]],
        }
        args.output.write_text(json.dumps(stats, indent=2)+'\n')
        print(json.dumps({k:v for k,v in stats.items() if k!='rss_kib'},indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
        log.close()


if __name__ == '__main__':
    main()
