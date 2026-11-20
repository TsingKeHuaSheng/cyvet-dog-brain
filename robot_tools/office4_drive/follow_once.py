#!/usr/bin/env python3
"""Plan a short forward trial or execute one freshly selected RViz preview path."""
import argparse
import fcntl
import math
import time
import sys
import signal

import rclpy
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, OccupancyGrid
from nav2_msgs.action import ComputePathToPose, FollowPath
from interface.srv import IsValid
from action_msgs.msg import GoalStatus


class Trial(Node):
    def __init__(self, drive):
        super().__init__('office4_trial')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.plan = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.follow = ActionClient(self, FollowPath, '/office4_drive/follow_path')
        self.check = self.create_client(IsValid, '/localizer/relocalize_check')
        self.enabled = False
        self.fault = None
        self.path = None
        self.latest = {}
        self.out = self.create_publisher(Twist, '/cmd_vel', 1) if drive else None
        self.create_subscription(Twist, '/office4_drive/checked_velocity', self.forward, 1)
        self.create_subscription(Path, '/navigation_preview/path', self.new_path, 1)
        for key, cls, topic in [('near',LaserScan,'/office4_drive/near_scan'),
                                ('scan',LaserScan,'/office4_live/scan'),
                                ('costmap',OccupancyGrid,'/office4_drive/local_costmap/costmap')]:
            self.create_subscription(cls, topic, lambda m,k=key:self.observe(k,m), qos_profile_sensor_data)
        self.last_command = None
        self.max_forward = 0.0
        self.straight_start = None
        self.travel_limit = None
        self.previous_speed = 0.0
        self.integrated_distance = 0.0

    def observe(self, key, m):
        self.latest[key] = Time.from_msg(m.header.stamp).nanoseconds * 1e-9

    def new_path(self, msg):
        self.path = msg

    def forward(self, msg):
        if not self.enabled:
            return
        x,y,w = msg.linear.x,msg.linear.y,msg.angular.z
        now = time.monotonic()
        self.integrated_distance += self.previous_speed * (now-self.last_command)
        self.last_command = now
        self.fault = self.fault or self.problem()
        if not all(map(math.isfinite,(x,y,w))) or not (-0.0001 <= x <= 0.0801 and abs(y)<0.0001 and abs(w)<=0.201):
            self.fault = self.fault or 'unexpected controller command'
        if not self.fault and self.straight_start is not None:
            t = self.transform().transform
            p,forward,yaw0 = self.straight_start
            dx,dy = t.translation.x-p[0],t.translation.y-p[1]
            q=t.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            cross=abs(-forward[1]*dx+forward[0]*dy)
            heading=abs(math.remainder(yaw-yaw0,2*math.pi))
            if cross>0.10 or heading>0.18 or abs(w)>0.12:
                self.fault='straight-trial alignment limit'
            if math.hypot(dx,dy)>=self.travel_limit or self.integrated_distance>=self.travel_limit:
                self.fault='straight-trial stopping margin reached'
        if self.fault:
            self.previous_speed = 0.0
            self.out.publish(Twist())
            return
        self.previous_speed = max(0.,x)
        self.max_forward = max(self.max_forward,x)
        self.out.publish(msg)

    def transform(self):
        return self.buffer.lookup_transform('map','base_link',Time())

    def problem(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        # 0.08 m/s with 1.0 s pose age already amounts to 8 cm of stale travel.
        for key,limit in [('near',0.60),('scan',1.0),('costmap',0.8)]:
            age=now-self.latest.get(key,0)
            if age < -0.15 or age>limit: return key+' stale or missing'
        try:
            age=now-Time.from_msg(self.transform().header.stamp).nanoseconds*1e-9
        except TransformException:
            return 'map/base_link unavailable'
        if not -0.15 <= age <= 1.0: return 'map/base_link stale'
        return None

    def wait(self,future,timeout):
        rclpy.spin_until_future_complete(self,future,timeout_sec=timeout)
        if not future.done():raise RuntimeError('ROS request timed out')
        return future.result()

    def get_path(self,ahead):
        if ahead is None:
            print('Select one NEW 2D Goal Pose in the Office4 RViz window.',flush=True)
            self.path=None; deadline=time.monotonic()+60
            while time.monotonic()<deadline:
                rclpy.spin_once(self,timeout_sec=.1)
                if self.path and self.path.poses:return self.path
            raise RuntimeError('No new nonempty path received')
        t=self.transform().transform; q=t.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        pose=PoseStamped();pose.header.frame_id='map'
        p=pose.pose.position;p.x=t.translation.x+ahead*math.cos(yaw);p.y=t.translation.y+ahead*math.sin(yaw)
        pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2)
        if not self.plan.wait_for_server(timeout_sec=4):raise RuntimeError('Planner unavailable')
        g=ComputePathToPose.Goal();g.goal=pose;g.planner_id='GridBased';g.use_start=False
        h=self.wait(self.plan.send_goal_async(g),5)
        if not h.accepted:raise RuntimeError('Planner rejected goal')
        r=self.wait(h.get_result_async(),10)
        if r.status!=GoalStatus.STATUS_SUCCEEDED:raise RuntimeError('Planner could not find this route')
        return r.result.path

    def run(self,args):
        deadline=time.monotonic()+3.0
        while time.monotonic()<deadline:rclpy.spin_once(self,timeout_sec=.08)
        if not self.check.wait_for_service(timeout_sec=3) or not self.wait(self.check.call_async(IsValid.Request(code=0)),4).valid:
            raise RuntimeError('Office4 localization is not valid')
        path=self.get_path(args.ahead)
        ps=[(s.pose.position.x,s.pose.position.y) for s in path.poses]
        if len(ps)<2 or path.header.frame_id!='map':raise RuntimeError('Missing map-frame route')
        length=sum(math.dist(a,b) for a,b in zip(ps,ps[1:]))
        t=self.transform().transform.translation
        print(f'Robot ({t.x:.2f}, {t.y:.2f}), path {ps[0]} -> {ps[-1]}, {length:.2f} m, {len(ps)} poses; readiness: {self.problem() or "ready"}',flush=True)
        if not args.drive:return
        if length>1.35 or math.dist(ps[0],(t.x,t.y))>.16:raise RuntimeError('Choose a short fresh route from current position for this trial')
        if args.ahead is not None:
            tf=self.transform().transform
            q=tf.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            fx,fy=math.cos(yaw),math.sin(yaw)
            p=(tf.translation.x,tf.translation.y)
            for px,py in ps:
                dx,dy=px-p[0],py-p[1]
                if abs(-fy*dx+fx*dy)>.08 or not -.05<=fx*dx+fy*dy<=args.ahead+.04:
                    raise RuntimeError('Route leaves the confirmed straight corridor')
            self.straight_start=(p,(fx,fy),yaw)
            self.travel_limit=args.ahead-.06
        if self.problem():raise RuntimeError(self.problem())
        if self.out.get_subscription_count()!=1:raise RuntimeError('Expected one SDK velocity relay')
        if not self.follow.wait_for_server(timeout_sec=3):raise RuntimeError('Tracker missing')
        g=FollowPath.Goal();g.path=path;g.controller_id='FollowPath';g.goal_checker_id='goal_checker'
        h=self.wait(self.follow.send_goal_async(g),3)
        if not h.accepted:raise RuntimeError('Tracking request rejected')
        f=h.get_result_async()
        try:
            self.enabled=True; self.last_command=time.monotonic()
            deadline=time.monotonic()+28
            while not f.done():
                rclpy.spin_once(self,timeout_sec=.07)
                error=self.fault or self.problem()
                if error or time.monotonic()>deadline or time.monotonic()-self.last_command>1.2:
                    raise RuntimeError(error or 'Tracking timed out')
            print('FollowPath result',f.result().status,'peak commanded forward speed',round(self.max_forward,3),flush=True)
            if f.result().status!=GoalStatus.STATUS_SUCCEEDED:raise RuntimeError('Tracker stopped before goal')
        finally:
            self.enabled=False
            self.out.publish(Twist())
            print(f'Peak sent {self.max_forward:.3f} m/s; integrated forward command {self.integrated_distance:.3f} m',flush=True)
            if not f.done():
                try:self.wait(h.cancel_goal_async(),1)
                except Exception:pass
            for _ in range(3):
                self.out.publish(Twist());time.sleep(.05)

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--ahead',type=float,help='Plan this distance along current heading instead of waiting for an RViz goal')
    parser.add_argument('--drive',action='store_true',help='Enable one real, short trial')
    args=parser.parse_args()
    if args.ahead is not None and not 0.15<=args.ahead<=1.2:parser.error('Choose 0.15 to 1.2 m')
    with open('/tmp/office4-follow.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        def interrupt(signum, frame):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, interrupt)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        n=Trial(args.drive)
        try:n.run(args)
        except (KeyboardInterrupt,RuntimeError,TransformException) as e:
            print('Trial stopped:',e,flush=True);sys.exit(1)
        finally:
            n.destroy_node()
            if rclpy.ok():rclpy.shutdown()
