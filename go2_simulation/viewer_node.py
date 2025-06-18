import rclpy
import meshcat

import numpy as np
import pinocchio as pin

from rclpy.node import Node
from nav_msgs.msg import Odometry
from unitree_go.msg import LowState
from pinocchio.visualize import MeshcatVisualizer
from go2_simulation.utils import loadGo2Model, addFloor


class ViewerNode(Node,):
    def __init__(self):
        Node.__init__(self, "viewer")

        model, geom_model = loadGo2Model()
        visual_model = geom_model.copy()
        addFloor(geom_model, visual_model)

        q = pin.neutral(model)

        viewer = meshcat.Visualizer()
        viewer.delete()
        self.vizer: MeshcatVisualizer = MeshcatVisualizer(model, geom_model, visual_model)
        self.vizer.initViewer(viewer=viewer, open=False, loadModel=True)
        self.vizer.display(q)

        self.create_subscription(LowState, "/lowstate", self.receive_state_cb, 10)
        self.last_state_msg = LowState()

        self.create_subscription(Odometry, "/odometry/filtered", self.receive_odometry_cb, 10)
        self.last_odometry_msg = Odometry()

        fps = 30
        period = 1./fps  # seconds
        self.timer = self.create_timer(period, self.update)

    def update(self):
        joint_pos = [self.last_state_msg.motor_state[i].q for i in range(12)]
        pos_msg = self.last_odometry_msg.pose.pose.position
        ff_pos = [pos_msg.x, pos_msg.y, pos_msg.z]
        quat_msg = self.last_odometry_msg.pose.pose.orientation
        ff_quat = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
        q = np.array(ff_pos + ff_quat + joint_pos)
        self.vizer.display(q)

    def receive_state_cb(self, msg):
        self.last_state_msg = msg

    def receive_odometry_cb(self, msg):
        self.last_odometry_msg = msg


def main(args=None):
    rclpy.init(args=args)
    node = ViewerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()