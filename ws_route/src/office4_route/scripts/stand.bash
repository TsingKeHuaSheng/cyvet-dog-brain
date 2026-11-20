#!/usr/bin/env bash
set -e
systemctl --user start uniubi-motion-bridge
source ~/robot_tools/uniubi_motion/dog_env.bash
/usr/bin/python3 - <<'PY'
import json
import rclpy
from std_srvs.srv import Trigger
from uniubi_motion_bridge.srv import StartMotionAction
rclpy.init()
n=rclpy.create_node('office4_prepare_walking')
def call(name,kind,req):
    client=n.create_client(kind,name)
    if not client.wait_for_service(timeout_sec=5):raise SystemExit('Missing '+name)
    f=client.call_async(req)
    rclpy.spin_until_future_complete(n,f,timeout_sec=10)
    if not f.done():raise SystemExit('Timeout: '+name)
    r=f.result()
    print(name,r.message,flush=True)
    if not r.success:raise SystemExit(1)
call('/motion/acquire_control',Trigger,Trigger.Request())
req=StartMotionAction.Request()
req.action='walking'
req.params_json=json.dumps(dict(lineVelocityX=0.0,lineVelocityY=0.0,velocity=0.0))
call('/motion/start_action',StartMotionAction,req)
n.destroy_node()
rclpy.shutdown()
PY
