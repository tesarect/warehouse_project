from nav_msgs.msg import Odometry
from threading import Lock as threadLock

class OdomListener:
    def __init__(self, node, topic='/odom'):
        self.node = node
        self.topic = topic
        self.current_pose = None
        self.lock = threadLock()
        
        # Subscribe to odometry
        self.odom_sub = node.create_subscription(
            Odometry,
            self.topic,
            self.odom_callback,
            10
        )
    
    def odom_callback(self, msg):
        with self.lock:
            self.current_pose = msg.pose.pose
    
    def get_current_pose(self):
        with self.lock:
            return self.current_pose