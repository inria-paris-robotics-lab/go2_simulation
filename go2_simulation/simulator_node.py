import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState, LowCmd
from nav_msgs.msg import Odometry
import numpy as np

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class Go2Simulator(Node):
    def __init__(self):
        super().__init__('go2_simulation')
        simulator_name = self.declare_parameter('simulator', rclpy.Parameter.Type.STRING).value

        ########################### State
        self.lowstate_publisher = self.create_publisher(LowState, "/lowstate", 10)
        self.odometry_publisher = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to publish periodically
        self.high_level_period = 1./500  # seconds
        self.low_level_sub_step = 12
        self.timer = self.create_timer(self.high_level_period, self.update)

        ########################## Cmd
        self.create_subscription(LowCmd, "/lowcmd", self.receive_cmd_cb, 10)
        self.last_cmd_msg = LowCmd()

        ########################## Simulator
        self.get_logger().info("go2_simulator::loading simulator")
        timestep = self.high_level_period / self.low_level_sub_step

        if simulator_name == "simple":
            from go2_simulation.simple_wrapper import SimpleWrapper
            self.simulator = SimpleWrapper(self, timestep)
        elif simulator_name == "pybullet":
            from go2_simulation.bullet_wrapper import BulletWrapper
            self.simulator = BulletWrapper(timestep)
        else:
            self.get_logger().error("Simulation tool not recognized")


    def update(self):
        low_msg = LowState()
        odometry_msg = Odometry()
        transform_msg = TransformStamped()

        timestamp = self.get_clock().now().to_msg()

        # Read sensors
        q_current, v_current = self.simulator.get_state()
        for joint_idx in range(self.simulator.njoints):
            low_msg.motor_state[joint_idx].mode = 1
            low_msg.motor_state[joint_idx].q = q_current[7 + joint_idx]
            low_msg.motor_state[joint_idx].dq = v_current[6 + joint_idx]
        # Read IMU
        low_msg.imu_state.quaternion = q_current[3:7].tolist()

        # Robot state
        self.lowstate_publisher.publish(low_msg)

        # Odometry / state estimation
        odometry_msg.header.stamp = timestamp
        odometry_msg.header.frame_id = "odom"
        odometry_msg.child_frame_id = "base"
        odometry_msg.pose.pose.position.x = q_current[0]
        odometry_msg.pose.pose.position.y = q_current[1]
        odometry_msg.pose.pose.position.z = q_current[2]
        odometry_msg.pose.pose.orientation.x = q_current[3]
        odometry_msg.pose.pose.orientation.y = q_current[4]
        odometry_msg.pose.pose.orientation.z = q_current[5]
        odometry_msg.pose.pose.orientation.w = q_current[6]
        odometry_msg.twist.twist.linear.x = v_current[0]
        odometry_msg.twist.twist.linear.y = v_current[1]
        odometry_msg.twist.twist.linear.z = v_current[2]
        odometry_msg.twist.twist.angular.x = v_current[3]
        odometry_msg.twist.twist.angular.y = v_current[4]
        odometry_msg.twist.twist.angular.z = v_current[5]
        self.odometry_publisher.publish(odometry_msg)

        # Forwar odometry on tf
        transform_msg.header.stamp = timestamp
        transform_msg.header.frame_id = "odom"
        transform_msg.child_frame_id = "base"
        transform_msg.transform.translation.x = q_current[0]
        transform_msg.transform.translation.y = q_current[1]
        transform_msg.transform.translation.z = q_current[2]
        transform_msg.transform.rotation.x = q_current[3]
        transform_msg.transform.rotation.y = q_current[4]
        transform_msg.transform.rotation.z = q_current[5]
        transform_msg.transform.rotation.w = q_current[6]
        self.tf_broadcaster.sendTransform(transform_msg)

        q_des   = np.array([self.last_cmd_msg.motor_cmd[i].q   for i in range(12)])
        v_des   = np.array([self.last_cmd_msg.motor_cmd[i].dq  for i in range(12)])
        tau_des = np.array([self.last_cmd_msg.motor_cmd[i].tau for i in range(12)])
        kp_des  = np.array([self.last_cmd_msg.motor_cmd[i].kp  for i in range(12)])
        kd_des  = np.array([self.last_cmd_msg.motor_cmd[i].kd  for i in range(12)])

        for _ in range(self.low_level_sub_step):
            self.simulator.execute_step(tau_des, q_des, v_des, kp_des, kd_des)

    def receive_cmd_cb(self, msg):
        self.last_cmd_msg = msg

def main(args=None):
    rclpy.init(args=args)
    try:
        go2_simulation = Go2Simulator()
        rclpy.spin(go2_simulation)
    except rclpy.exceptions.ROSInterruptException:
        pass

    go2_simulation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

