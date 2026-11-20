import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Header
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from route_map import ROUTE, cloud_message, read_map, route_message, load_route, save_route


class Editor(Node):
    def __init__(self):
        super().__init__('office4_route_editor')
        p,self.fingerprint = read_map()
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        background = '/line_route/pointcloud' if 'DOG_ROUTE_MAP' in os.environ else '/line_route/office4'
        self.bg=self.create_publisher(PointCloud2,background,qos)
        self.route=self.create_publisher(Path,'/line_route/saved',qos)
        # The roof would obscure the floor plan in an overhead view.
        floor = self.declare_parameter('floor_z', -0.52).value
        visible=p[(p[:,2]>=floor-0.12)&(p[:,2]<=floor+1.0)]
        self.bg.publish(cloud_message(Header(frame_id='map'),visible))
        if ROUTE.exists():
            try:
                self.route.publish(load_route(ROUTE,self.fingerprint))
            except (ValueError,KeyError,OSError) as err:
                self.get_logger().error(str(err))
        self.create_subscription(Path,'/line_route/drawn',self.save,1)

    def save(self,msg):
        if msg.header.frame_id!='map':
            return
        xy=[(p.pose.position.x,p.pose.position.y) for p in msg.poses]
        try:
            path=route_message(xy)
            save_route(ROUTE,xy,self.fingerprint)
        except (ValueError,OSError) as err:
            self.get_logger().error(str(err))
            return
        self.route.publish(path)
        self.get_logger().info(f'Saved {len(xy)} drawn points to {ROUTE}')


if __name__=='__main__':
    rclpy.init()
    rclpy.spin(Editor())
