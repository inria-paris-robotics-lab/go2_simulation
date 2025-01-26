import pybullet
import pybullet_data
import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState, LowCmd

import numpy as np
from example_robot_data import getModelPath
import os

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class Go2Simulator(Node):
    def __init__(self):
        super().__init__('go2_simulation')
        
        # self.q0 = [0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1.0, 1.0, -1.5, -1.5, -1.5, -1.5]
        self.q0 = [0.0, 1.00, -2.51, 0.0, 1.09, -2.61, 0.2, 1.19, -2.59, -0.2, 1.32, -2.79] # Bullet order

        ########################### State
        self.last_cmd_msg = LowCmd()
        self.lowstate_publisher = self.create_publisher(LowState, "/lowstate", 10)


        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to publish periodically
        self.high_level_period = 1./500  # seconds
        self.low_level_sub_step = 12

        ########################## Cmd
        self.create_subscription(LowCmd, "/lowcmd", self.receive_cmd_cb, 10)

        robot_subpath = "go2_description/urdf/go2.urdf"
        self.robot_path = os.path.join(getModelPath(robot_subpath), robot_subpath)
        self.robot = 0
        self.init_pybullet()

        self.i = 0

        self.timer = self.create_timer(self.high_level_period, self.update)
        self.last_lin_vel = np.zeros(3, dtype=np.float32)
        self.last_lin_acc = np.zeros(3, dtype=np.float32)


    def init_pybullet(self):
        cid = pybullet.connect(pybullet.SHARED_MEMORY)
        self.get_logger().info(f"go2_simulator::pybullet:: cid={cid} ")
        if (cid < 0):
            pybullet.connect(pybullet.GUI, options="--opengl3")
        else:
            pybullet.connect(pybullet.GUI)

        # Load robot
        self.get_logger().info(f"go2_simulator::loading urdf : {self.robot_path}")
        self.robot = pybullet.loadURDF(self.robot_path, [0, 0, 0.4])
        self.get_logger().info(f"go2_simulator::loading urdf : {self.robot}")

        # Print joint names
        num_joints = pybullet.getNumJoints(self.robot)
        feet_names = [name + '_foot' for name in ("FL", "FR", "RL", "RR")]
        self.feet_idx = []

        for i in range(num_joints):
            joint_info = pybullet.getJointInfo(self.robot, i)
            joint_name = joint_info[1].decode('utf-8')
            link_name = joint_info[12].decode('utf-8')
            self.get_logger().info(f"go2_simulator::joint_info : {joint_name}")

            if link_name in feet_names:
                self.feet_idx.append((i, link_name))
    

        # Load ground plane
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = pybullet.loadURDF("plane.urdf")
        pybullet.resetBasePositionAndOrientation(self.plane_id, [0, 0, 0], [0, 0, 0, 1])

        pybullet.setTimeStep(self.high_level_period / self.low_level_sub_step)

        UNITREE_ORDER = ["FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"]
        ISAAC_ORDER = ["FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint", "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint", "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint"]
        self.joint_order = UNITREE_ORDER

        self.j_idx = []
        for j in self.joint_order:
            self.j_idx.append(self.get_joint_id(j))


        for i, id in enumerate(self.j_idx):
            pybullet.resetJointState(self.robot, id, self.q0[i], 0.0)

        # gravity and feet friction
        pybullet.setGravity(0, 0, -9.81)

        # Somehow this disable joint friction
        pybullet.setJointMotorControlArray(
            bodyIndex=self.robot,
            jointIndices=self.j_idx,
            controlMode=pybullet.VELOCITY_CONTROL,
            targetVelocities=[0. for i in range(12)],
            forces=[0. for i in range(12)],
        )

    def update(self):
        low_msg = LowState()

        timestamp = self.get_clock().now().to_msg()

        # Read sensors
        joint_states = pybullet.getJointStates(self.robot, self.j_idx)
        received_q = [joint_state[0] for joint_state in joint_states]

        for joint_idx, joint_state in enumerate(joint_states):
            low_msg.motor_state[joint_idx].mode = 1
            low_msg.motor_state[joint_idx].q = joint_state[0]
            low_msg.motor_state[joint_idx].dq = joint_state[1]

        # Read IMU
        position, orientation = pybullet.getBasePositionAndOrientation(self.robot) # world frame
        linear_vel, angular_vel = pybullet.getBaseVelocity(self.robot) # world frame

        # Convert to body frame
        R = np.array(pybullet.getMatrixFromQuaternion(orientation), dtype=np.float32).reshape(3, 3)
        linear_vel = np.dot(R.T, linear_vel).astype(np.float32)
        angular_vel = np.dot(R.T, angular_vel).astype(np.float32)

        low_msg.imu_state.quaternion = orientation
        low_msg.imu_state.gyroscope = angular_vel

        new_lin_acc = np.zeros(3, dtype=np.float32)
        new_lin_acc[:] = (np.array(linear_vel) - self.last_lin_vel) / self.high_level_period
        new_lin_acc[:] = new_lin_acc * 0.025 + self.last_lin_acc * 0.975

        # Get gravity vector from orientation
        gravity = np.array([0, 0, +9.81])

        gravity = np.dot(R, gravity)

        self.last_lin_acc[:] = new_lin_acc
        self.last_lin_vel[:] = linear_vel
        new_lin_acc += gravity
        low_msg.imu_state.accelerometer[:] = new_lin_acc
        norm = np.linalg.norm(new_lin_acc)

        # Update feet contact states
        for i, (joint_idx, link_name) in enumerate(self.feet_idx):
            contact_points = pybullet.getClosestPoints(self.robot, self.plane_id, 0.01, joint_idx)
            if len(contact_points) > 0:
                low_msg.foot_force[i] = 100

        # Robot state
        self.lowstate_publisher.publish(low_msg)

        q_des   = np.array([self.last_cmd_msg.motor_cmd[i].q   for i in range(12)])
        v_des   = np.array([self.last_cmd_msg.motor_cmd[i].dq  for i in range(12)]) * 0 # No velocity control
        tau_des = np.array([self.last_cmd_msg.motor_cmd[i].tau for i in range(12)]) * 0 # No torque control
        kp_des  = np.array([self.last_cmd_msg.motor_cmd[i].kp  for i in range(12)])
        kd_des  = np.array([self.last_cmd_msg.motor_cmd[i].kd  for i in range(12)])

        for _ in range(self.low_level_sub_step):
            # Get sub step state
            joint_states = pybullet.getJointStates(self.robot, self.j_idx)
            q = np.array([joint_state[0] for joint_state in joint_states])
            v = np.array([joint_state[1] for joint_state in joint_states])

            tau_cmd = tau_des - np.multiply(q-q_des, kp_des) - np.multiply(v-v_des, kd_des)
            # Clip torque command
            tau_cmd = np.clip(tau_cmd, -25, 25) # Nm, torque limit

            # Set actuation
            pybullet.setJointMotorControlArray(
                bodyIndex=self.robot,
                jointIndices=self.j_idx,
                controlMode=pybullet.TORQUE_CONTROL,
                forces=tau_cmd
            )

            # Advance simulation by one step
            pybullet.stepSimulation()

    def receive_cmd_cb(self, msg):
        self.last_cmd_msg = msg

    def get_joint_id(self, joint_name):
        num_joints = pybullet.getNumJoints(self.robot)
        for i in range(num_joints):
            joint_info = pybullet.getJointInfo(self.robot, i)
            if joint_info[1].decode("utf-8") == joint_name:
                return i
        return None  # Joint name not found

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

