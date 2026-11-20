import json
import time
import sys
import rclpy
from uniubi_motion_bridge.msg import MotionStatus
from uniubi_motion_bridge.srv import JsonCommand

rclpy.init()
n=rclpy.create_node('route_robot_state')
control=[]
sub=n.create_subscription(MotionStatus,'/motion/status',lambda m:control.append(m.control_state),1)
c=n.create_client(JsonCommand,'/motion/query_state')
if not c.wait_for_service(timeout_sec=3):sys.exit('Robot bridge is unavailable')
f=c.call_async(JsonCommand.Request())
rclpy.spin_until_future_complete(n,f,timeout_sec=6)
deadline=time.monotonic()+2
while not control and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.1)
if not f.done() or not f.result().success:sys.exit('Robot state unavailable')
s=json.loads(f.result().result_json)
if not control or control[-1]!=2 or s.get('action')!='walking':
    sys.exit('Prepare robot: acquire control and stand in zero-speed walking mode first.')
if any(abs(float(s.get(k,0)))>0.001 for k in ('lineVelocityX','lineVelocityY','velocity')):
    sys.exit('Start from a stationary robot.')
n.destroy_node()
rclpy.shutdown()
