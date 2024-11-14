import pybullet as p
import pybullet_data
import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState, LowCmd
import time
from example_robot_data import getModelPath
import os


class Go2Simulator(Node):
    def __init__(self):
        super().__init__('go2_simulation')

        ########################### State
        self.publisher_state = self.create_publisher(LowState, "/lowstate", 10)

        # Timer to publish periodically
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.update)

        ########################## Cmd
        self.create_subscription(LowCmd, "/lowcmd", self.receive_cmd_cb, 10)

        robot_subpath = "go2_description/urdf/go2.urdf"
        self.robot_path = os.path.join(getModelPath(robot_subpath), robot_subpath)
        self.robot = 0
        self.init_pybullet()
        self.last_cmd_msg = LowCmd()

    def init_pybullet(self):
        cid = p.connect(p.SHARED_MEMORY)
        self.get_logger().info(f"go2_simulator::pybullet:: cid={cid} ")
        if (cid < 0):
            p.connect(p.GUI, options="--opengl2")
        else:
            p.connect(p.GUI)

        self.get_logger().info(f"go2_simulator::loading urdf : {self.robot_path}")
        self.robot = p.loadURDF(self.robot_path, [0, 0, 0.45])
        p.setGravity(0, 0, -9.81)

        # Load plane and robot
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = p.loadURDF("plane.urdf")
        p.resetBasePositionAndOrientation(self.plane_id, [0, 0, 0], [0, 0, 0, 1])
        for _ in range(40):
            p.stepSimulation()

        self.joint_order = ["FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"]

        self.j_idx = []
        for j in self.joint_order:
            self.j_idx.append(self.get_joint_id(j))

    def update(self):
        state_msg = LowState()

        for joint_idx in range(12):
            # Read sensors
            joint_state = p.getJointState(self.robot, self.j_idx[joint_idx])
            state_msg.motor_state[joint_idx].mode = 1
            state_msg.motor_state[joint_idx].q = joint_state[0]
            state_msg.motor_state[joint_idx].dq = joint_state[1]

            # Set Actuation
            target_position = self.last_cmd_msg.motor_cmd[joint_idx].q
            target_velocity = self.last_cmd_msg.motor_cmd[joint_idx].dq
            j_id = self.j_idx[joint_idx]

            p.setJointMotorControl2(
                bodyIndex=self.robot,
                jointIndex=j_id,
                controlMode=p.POSIITON_CONTROL,
                targetPosition=target_position,
                targetVelocity=target_velocity
            )

        # Read IMU
        position, orientation = p.getBasePositionAndOrientation(self.robot)
        state_msg.imu_state.quaternion = orientation
        self.publisher_state.publish(state_msg)

        # Advance simulation by one step
        p.stepSimulation()

    def receive_cmd_cb(self, msg):
        self.last_cmd_msg = msg

    def get_joint_id(self, joint_name):
        num_joints = p.getNumJoints(self.robot)
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot, i)
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

