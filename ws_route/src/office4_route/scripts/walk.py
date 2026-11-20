"""Execute the saved stroke once; drawing a stroke never invokes this action."""
import argparse
import fcntl
import math
import signal
import time
import sys
from collections import deque
from pathlib import Path as FilePath

import numpy as np
import yaml
from scipy.spatial import cKDTree
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped, Twist
from interface.srv import IsValid
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from detection_zone import route_id, route_xy
from tf2_ros import Buffer, TransformListener, TransformException
from route_map import MAP_NAME, ROUTE, read_map, load_route, xyz
from route_tracking import Alignment, MotionProgress, RouteSection, stamp_seconds


def sections(path):
    # A goal checker can finish too early if a loop visits its endpoint twice.
    pending = [path.poses]
    while pending:
        poses = pending.pop()
        xy = np.array([(p.pose.position.x,p.pose.position.y) for p in poses])
        arc = np.r_[0.,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
        hits = np.flatnonzero((arc[-1]-arc>0.28)&(np.linalg.norm(xy-xy[-1],axis=1)<0.12))
        if hits.size:
            cut = int(np.searchsorted(arc, (arc[hits[-1]]+arc[-1])/2))
            if 0 < cut < len(poses)-1:
                pending.extend([poses[cut:],poses[:cut+1]])
                continue
        section=Path()
        section.header.frame_id='map'
        section.poses=poses
        yield section


def remaining_route(path, x, y):
    """Resume on one unambiguous stretch of the already densified stroke."""
    xy = np.array([(p.pose.position.x, p.pose.position.y) for p in path.poses])
    distances = np.linalg.norm(xy - [x, y], axis=1)
    nearest = int(np.argmin(distances))
    if distances[nearest] > 0.35:
        raise RuntimeError(f'距原线 {distances[nearest]:.2f} 米，超出 35 厘米的纠回范围')
    # Reject separated passes near the robot, including the two sides of a crossing.
    nearby = np.flatnonzero(distances <= distances[nearest] + 0.055)
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    if np.any(np.diff(nearby) > 1) or arc[nearby[-1]] - arc[nearby[0]] > 0.45:
        raise RuntimeError('此处有多段紫线靠得太近，请移到能区分前后方向的单段紫线上再续走')
    index = min(nearest, len(path.poses) - 2)
    result = Path()
    result.header = path.header
    result.poses = path.poses[index:]
    print(f'沿原线续走：距 A 的沿线里程 {arc[index]:.2f} 米，剩余 {arc[-1]-arc[index]:.2f} 米', flush=True)
    return result, float(arc[index])


class Walk(Node):
    def __init__(self):
        super().__init__('office4_route_run')
        config = FilePath(__file__).resolve().parents[1] / 'config' / 'controller.yaml'
        with config.open() as stream:
            settings = yaml.safe_load(stream)
        params = settings['/line_route/controller_server']['ros__parameters']
        self.sensor_timeout = settings['/office4_obstacle_gate']['ros__parameters']['source_timeout']
        self.max_forward_speed = params['FollowPath']['desired_linear_vel']
        self.progress_radius = params['progress_checker']['required_movement_radius']
        self.progress_seconds = params['progress_checker']['movement_time_allowance']
        self.progress_angle = params['progress_checker']['required_movement_angle']
        self.lookahead = params['FollowPath']['lookahead_dist']
        self.goal_tolerance = params['goal_checker']['xy_goal_tolerance']
        if type(self.max_forward_speed) not in (float, int) or not (
            math.isfinite(self.max_forward_speed) and self.max_forward_speed > 0
        ):
            raise ValueError(f'Invalid FollowPath.desired_linear_vel in {config}')
        points,fingerprint=read_map()
        self.route=load_route(ROUTE,fingerprint)
        self.map_tree=cKDTree(points)
        self.map_matches=None
        self.tf=Buffer()
        self.listener=TransformListener(self.tf,self)
        self.client=ActionClient(self,FollowPath,'/line_route/follow_path')
        self.valid=self.create_client(IsValid,'/localizer/relocalize_check')
        self.output=self.create_publisher(Twist,'/cmd_vel',1)
        self.create_subscription(PointCloud2,'/localizer/map_cloud',self.check_map,qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, '/line_route/obstacle_gate', self.observe, 1)
        self.create_subscription(Twist,'/line_route/tracking',self.command,1)
        self.create_subscription(Log, '/rosout', self.controller_log, 20)
        self.create_subscription(TransformStamped, '/localizer/accepted_alignment', self.align, 1)
        self.controller_error = ''
        self.route_key = None
        self.turning = False
        self.alignment = None
        self.candidate = None
        self.confirmations = 0
        self.confirmation_stamp = 0.
        self.alignment_origin = None
        self.local_pose = None
        self.local_position = None
        self.offset = self.total = self.mileage = 0.
        self.message = ''
        self.printed_at = 0.
        self.local_stamp = 0.
        self.scan_time = 0.
        self.scan_number = 0
        self.blocked = False
        self.last_command = 0.
        self.enabled = False
        self.started = False
        self.pause = ''
        self.fault = ''
        self.history = deque(maxlen=1200)
        self.relay_present = False
        self.relay_check_time = 0.

    def align(self, msg):
        if (msg.header.frame_id, msg.child_frame_id) != ('map', 'lidar'):
            self.alignment = None
            if self.started:
                self.fault = '定位正在切换，请核对路线后重新启动'
            return
        new = Alignment(msg)
        if new.stamp <= self.confirmation_stamp:
            return
        self.confirmation_stamp = new.stamp
        if self.started and self.alignment and self.local_position is not None:
            delta, angle = new.difference_at(self.alignment, self.local_position)
            if self.candidate is not None or delta > 0.20 or angle > 12:
                if self.candidate is None:
                    self.confirmations = 0
                else:
                    shift, turn = new.difference_at(self.candidate, self.local_position)
                    if shift > 0.06 or turn > 3 or new.stamp - self.candidate_stamp > 2.0:
                        self.confirmations = 0
                if self.confirmations == 0:
                    self.candidate = new
                self.candidate_stamp = new.stamp
                self.confirmations += 1
                if self.confirmations < 3:
                    if self.enabled:
                        self.pause = '地图匹配跳变，停车等待连续三次匹配一致'
                        self.output.publish(Twist())
                    return
                self.candidate = None
        self.alignment = new
        self.alignment_origin = None if self.local_position is None else self.local_position.copy()

    def controller_log(self, msg):
        if msg.name == 'line_route.controller_server' and msg.level >= LoggingSeverity.ERROR:
            self.controller_error = msg.msg

    def check_map(self,msg):
        if msg.header.frame_id!='map' or not msg.width:
            self.map_matches=False
            return
        pts=xyz(msg)
        if len(pts)<100:
            self.map_matches=False
            return
        distances=self.map_tree.query(pts[::max(1,len(pts)//700)])[0]
        self.map_matches=bool(np.percentile(distances,90)<0.12)

    def observe(self, msg):
        if msg.header.frame_id != 'map' or len(msg.status) != 1:
            return
        stamp = stamp_seconds(msg.header.stamp)
        if stamp <= self.scan_time:
            return
        result = msg.status[0]
        fields = {v.key: v.value for v in result.values}
        if result.name != 'office4_route/obstacle_gate':
            return
        if fields.get('route_id') != self.route_key:
            self.fault = '画线已更换，停车后请按新路线启动'
            if self.enabled:
                self.output.publish(Twist())
            return
        self.scan_time = stamp
        self.scan_number += 1
        self.blocked = result.level != DiagnosticStatus.OK
        if self.blocked and self.enabled:
            self.pause = '前方被占用，等待通道清空'
            self.output.publish(Twist())

    def require_inputs(self):
        if self.fault:
            raise RuntimeError(self.fault)
        if self.map_matches is False and self.started:
            raise RuntimeError(f'定位底图与 {MAP_NAME} 不匹配，请核对地图')
        if not self.map_matches:
            return f'等待 {MAP_NAME} 定位底图'
        clock = time.monotonic()
        if clock >= self.relay_check_time:
            self.relay_present = any(e.node_name == 'uniubi_velocity_link_0'
                                     for e in self.get_subscriptions_info_by_topic('/cmd_vel'))
            self.relay_check_time = clock + 0.25
        if not self.relay_present:
            if self.started:
                raise RuntimeError('速度转发已断开，请核对控制权后重新启动')
            return '等待速度转发节点就绪'
        now = self.get_clock().now().nanoseconds * 1e-9
        try:
            base = self.tf.lookup_transform('lidar', 'base_link', Time())
        except TransformException:
            return '等待 FAST-LIO2 里程计'
        t = base.transform.translation
        self.local_stamp = stamp_seconds(base.header.stamp)
        if not -0.1 <= now - self.local_stamp <= self.sensor_timeout:
            return '等待 FAST-LIO2 更新机身位置'
        self.local_position = np.array([t.x, t.y, t.z])
        self.local_pose = Alignment(base)
        if not -0.1 <= now - self.scan_time <= self.sensor_timeout:
            return '等待实时障碍点云'
        if self.blocked:
            return '前方被占用，等待通道清空'
        if self.candidate is not None:
            return '地图匹配跳变，停车等待连续三次匹配一致'
        if self.alignment is None or not -0.1 <= now - self.alignment.stamp <= 5.0:
            return '等待新的地图匹配'
        if self.alignment_origin is None:
            self.alignment_origin = self.local_position.copy()
        if np.linalg.norm(self.local_position - self.alignment_origin) > 1.0:
            return '等待新的地图匹配'
        return ''

    def command(self, msg):
        if not self.enabled:
            return
        self.last_command = time.monotonic()
        self.history.append((self.last_command, '跟踪器', msg.linear.x, msg.angular.z))
        x, y, w = msg.linear.x, msg.linear.y, msg.angular.z
        out = Twist()
        try:
            self.pause = self.pause or self.require_inputs()
            if not all(map(math.isfinite, (x,y,w))) or x < -1e-5 or abs(y) > 1e-5:
                raise RuntimeError('Unexpected tracking command')
            if not self.pause:
                scale = max(1., abs(x) / self.max_forward_speed, abs(w) / 0.20)
                out.linear.x, out.angular.z = x / scale, w / scale
        except RuntimeError as exc:
            self.fault = str(exc)
        if out.linear.x >= 0.001:
            self.turning = False
        elif abs(out.angular.z) > 0.005:
            self.turning = True
        self.output.publish(out)
        self.history.append((self.last_command, '/cmd_vel', out.linear.x, out.angular.z))

    def wait(self,f,t):
        rclpy.spin_until_future_complete(self,f,timeout_sec=t)
        if not f.done(): raise RuntimeError('Request timed out')
        return f.result()

    def status(self, message):
        now = time.monotonic()
        if message.startswith('正在') and self.message.startswith('正在') and now - self.printed_at < 2.0:
            return
        if message != self.message or now - self.printed_at >= 5.0:
            if self.mileage is None:
                print(f'{message}｜等待确定当前沿线里程，全长 {self.total:.2f} 米', flush=True)
            else:
                print(f'{message}｜距 A {self.mileage:.2f} 米，剩余 {max(0., self.total-self.mileage):.2f} 米', flush=True)
            self.message, self.printed_at = message, now

    def route_distance(self, section):
        distance = section.update(self.alignment.apply(self.local_position))
        self.mileage = self.offset + section.arc[section.index]
        return distance

    def wait_ready(self, section=None):
        clear_since = None
        last_zero = 0.
        while True:
            rclpy.spin_once(self, timeout_sec=0.08)
            reason = self.require_inputs()
            if not reason and section and self.route_distance(section) > 0.35:
                reason = '超出当前路段可纠回范围，停车等待回到原段附近'
            now = time.monotonic()
            if now >= last_zero + 0.15:
                self.output.publish(Twist())
                last_zero = now
            if reason:
                self.status(reason)
                clear_since = None
            elif clear_since is None:
                clear_since = now
                scan, odom = self.scan_number, self.local_stamp
            elif now - clear_since >= 0.5 and self.scan_number > scan and self.local_stamp > odom:
                return

    def velocity_summary(self, turning=False):
        samples = [row for row in self.history if row[0] >= time.monotonic()-25.]
        values = {}
        for source in ('跟踪器', '/cmd_vel'):
            subset = [row for row in samples if row[1] == source]
            values[source] = (max((abs(r[2]) for r in subset), default=0.),
                              max((abs(r[3]) for r in subset), default=0.))
        detail = '；'.join(f'{k} 峰值 {v:.3f} m/s、{w:.3f} rad/s'
                          for k, (v,w) in values.items())
        if turning:
            return f'已发转向指令，机身与路线的夹角未持续缩小；{detail}'
        if values['/cmd_vel'][0] > 0:
            return f'曾发前进指令，定位平移未达到进展要求；{detail}'
        return f'定位平移未达到进展要求；{detail}'

    def send_section(self, section, anchor):
        g = FollowPath.Goal()
        g.path = section.local_path(anchor)
        g.controller_id = 'FollowPath'
        g.goal_checker_id = 'goal_checker'
        handle = self.wait(self.client.send_goal_async(g), 4)
        if not handle.accepted:
            raise RuntimeError('控制器拒绝接收当前路线')
        return handle, handle.get_result_async()

    def follow(self, section):
        self.history.clear()
        while True:
            self.wait_ready(section)
            anchor = self.alignment
            self.pause = self.controller_error = ''
            self.turning = False
            handle, result = self.send_section(section, anchor)
            progress = MotionProgress(self.progress_radius, self.progress_angle)
            self.enabled = True
            self.last_command = previous_tick = updated = time.monotonic()
            try:
                while True:
                    rclpy.spin_once(self, timeout_sec=0.05)
                    reason = self.require_inputs() or self.pause
                    if reason:
                        break
                    distance = self.route_distance(section)
                    if distance > 0.35:
                        reason = '超出当前路段可纠回范围，停车等待回到原段附近'
                        break
                    tick = time.monotonic()
                    dt, previous_tick = tick - previous_tick, tick
                    turning = self.turning
                    yaw = section.turn_heading(self.local_pose, anchor, self.lookahead, self.goal_tolerance) if turning else None
                    progress.sample(self.local_pose, turning, yaw, dt)
                    state = '正在对准路线方向' if turning else '正在沿线行走'
                    if distance > 0.20:
                        state = '正在纠回当前路段' if not turning else '正在转向以纠回当前路段'
                    self.status(state)
                    if progress.seconds > self.progress_seconds and not result.done():
                        raise RuntimeError(self.velocity_summary(turning))
                    if result.done():
                        if result.result().status == GoalStatus.STATUS_SUCCEEDED:
                            position = self.alignment.apply(self.local_position)
                            if np.linalg.norm(section.xy[-1] - position[:2]) <= 0.12:
                                return
                            reason = '终点位置已更新，继续跟踪本段'
                            break
                        self.enabled = False
                        self.output.publish(Twist())
                        deadline = time.monotonic() + 0.25
                        while time.monotonic() < deadline:
                            rclpy.spin_once(self, timeout_sec=0.025)
                        if 'progress' in self.controller_error.lower():
                            raise RuntimeError(self.velocity_summary(turning))
                        raise RuntimeError(self.controller_error or 'Controller stopped before the goal')
                    if tick - self.last_command > 2.0:
                        raise RuntimeError('跟踪器停止输出速度')
                    delta, angle = anchor.difference_at(self.alignment, self.local_position)
                    if (delta > 0.06 or angle > 3) and tick - updated >= 0.8:
                        # Humble accepts a new FollowPath as a preemption. The
                        # obstacle gate remains active throughout the swap.
                        anchor = self.alignment
                        handle, result = self.send_section(section, anchor)
                        updated = tick
            finally:
                self.enabled = False
                self.output.publish(Twist())
                if not result.done():
                    self.wait(handle.cancel_goal_async(), 3)
                    self.wait(result, 4)
            self.status(reason)

    def run(self, resume=False):
        self.route_key = route_id(route_xy(self.route))
        self.total = RouteSection(self.route).arc[-1]
        self.mileage = None if resume else 0.
        self.wait_ready()
        if not self.valid.wait_for_service(timeout_sec=2) or not self.wait(
                self.valid.call_async(IsValid.Request(code=0)),3).valid:
            raise RuntimeError('定位尚未成功')
        self.wait_ready()
        position = self.alignment.apply(self.local_position)
        start = self.route.poses[0].pose.position
        if resume:
            self.route, self.offset = remaining_route(self.route, *position[:2])
            self.mileage = self.offset
        elif math.hypot(position[0]-start.x, position[1]-start.y) > 0.20:
            raise RuntimeError('请将路线起点 A 画在当前机身位置 20 厘米以内')
        if not self.client.wait_for_server(timeout_sec=3):
            raise RuntimeError('FollowPath is unavailable')
        self.started = True
        for section in sections(self.route):
            part = RouteSection(section)
            self.follow(part)
            self.offset += part.arc[-1]
            self.mileage = self.offset
        self.status('Arrived at B. 已到达 B')


if __name__=='__main__':
    parser = argparse.ArgumentParser(description='沿 Office4 中保存的手绘路线行走')
    parser.add_argument('--resume', action='store_true', help='从当前所在的原线路段向 B 续走')
    args = parser.parse_args()
    def interrupt(signum,frame): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupt)
    with open('/tmp/office4-follow.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node=None
        try:
            node=Walk()
            node.run(resume=args.resume)
        except KeyboardInterrupt:
            print('已手动停止循线', flush=True)
        except (RuntimeError,ValueError,OSError,KeyError,TransformException) as err:
            sys.exit(f'Stopped: {err}')
        finally:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            if node:
                node.enabled=False
                for _ in range(3):
                    node.output.publish(Twist())
                    time.sleep(.05)
                node.destroy_node()
            rclpy.shutdown()
