
import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState, LowCmd
from nav_msgs.msg import Odometry
import numpy as np
import pinocchio as pin
import example_robot_data
import os
from go2_simulation.simulation_args import SimulationArgs
from go2_simulation.simulation_utils import (
    addFloor,
    removeBVHModelsIfAny,
    setPhysicsProperties,
    Simulation,
    addSystemCollisionPairs
)
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class Go2Simulator(Node):
    def __init__(self):
        super().__init__('go2_simulation')

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
        
        ########################## Load robot model and geometry
        robot = example_robot_data.load("go2")
        self.rmodel = robot.model
        self.q0 = self.rmodel.referenceConfigurations["standing"]
        self.njoints = self.rmodel.nv - 6

        URDF_SUBPATH = "/go2_description/urdf/go2.urdf"
        package_dir = example_robot_data.getModelPath(URDF_SUBPATH)
        file_path = package_dir + URDF_SUBPATH

        with open(file_path, 'r') as file:
            file_content = file.read()

        self.geom_model = pin.GeometryModel()
        pin.buildGeomFromUrdfString(self.rmodel, file_content, pin.GeometryType.VISUAL, self.geom_model, package_dir)

        self.init_simple()
        self.last_cmd_msg = LowCmd()

    def init_simple(self):
        visual_model = self.geom_model.copy()
        addFloor(self.geom_model, visual_model)

        # Set simulation properties
        args = SimulationArgs()
        initial_q = np.array([0, 0, 0.2, 0, 0, 0, 1, 0.0, 1.00, -2.51, 0.0, 1.09, -2.61, 0.2, 1.19, -2.59, -0.2, 1.32, -2.79])
        setPhysicsProperties(self.geom_model, args.material, args.compliance)
        removeBVHModelsIfAny(self.geom_model)
        addSystemCollisionPairs(self.rmodel, self.geom_model, initial_q)

         # Remove all pair of collision which does not concern floor collision
        i = 0
        while i < len(self.geom_model.collisionPairs):
            cp = self.geom_model.collisionPairs[i]
            if self.geom_model.geometryObjects[cp.first].name != 'floor' and self.geom_model.geometryObjects[cp.second].name != 'floor':
                self.geom_model.removeCollisionPair(cp)
            else:
                i = i + 1
        
        # Create the simulator object
        self.simulator = Simulation(self.rmodel, self.geom_model, visual_model, initial_q, np.zeros(self.rmodel.nv), args) 


    def update(self):
        low_msg = LowState()
        odometry_msg = Odometry()
        transform_msg = TransformStamped()

        timestamp = self.get_clock().now().to_msg()

        # Read sensors
        q_current, v_current = self.simulator.get_state()
        for joint_idx in range(self.njoints):
            low_msg.motor_state[joint_idx].mode = 1
            low_msg.motor_state[joint_idx].q = q_current[7 + joint_idx]
            low_msg.motor_state[joint_idx].dq = v_current[6 + joint_idx]
        # Read IMU
        low_msg.imu_state.quaternion = q_current[3:7].tolist()

        # Publish robot joint state
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
            # Get sub step state
            q_current, v_current = self.simulator.get_state()
            
            tau_cmd = tau_des - np.multiply(q_current[7:]-q_des, kp_des) - np.multiply(v_current[6:]-v_des, kd_des)

            # Set actuation and run one step of simulation
            torque_simu = np.zeros(self.rmodel.nv)
            torque_simu[6:] = tau_cmd
            self.simulator.execute(torque_simu)


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

