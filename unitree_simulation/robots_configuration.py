import os

from unitree_description import (
    GO2_DESCRIPTION_URDF_PATH,
    G1_DESCRIPTION_URDF_PATH,
    G1_DESCRIPTION_MODEL_DIR,
)
from abc import ABC, abstractmethod
from unitree_go.msg import LowState as LowStateGo2, LowCmd as LowCmdGo2
from unitree_hg.msg import LowState as LowStateG1, LowCmd as LowCmdG1
from typing import List


class RobotConfigurationAbstract(ABC):
    @property
    @abstractmethod
    def high_level_period(self) -> float:
        """Control loop period exposed to user."""
        pass

    @property
    @abstractmethod
    def low_level_sub_step(self) -> int:
        """Number of low-level steps per high-level step. (internal controller of the motors)"""
        pass

    @property
    def sim_dt(self) -> int:
        return self.high_level_period / self.low_level_sub_step

    @property
    @abstractmethod
    def urdf_path(self) -> str:
        """Path to the robot's URDF description file."""
        pass

    @property
    @abstractmethod
    def q_start(self) -> List[float]:
        """Initial configuration vector [base position, orientation quaternion, joint angles]."""
        pass

    @property
    @abstractmethod
    def joint_names(self) -> List[str]:
        """Ordered list of joint names."""
        pass

    @property
    @abstractmethod
    def feet_sensors_names(self) -> List[str]:
        """List of foot sensor names, if any."""
        pass

    @property
    @abstractmethod
    def lowstate_msgs_type(self):
        pass

    @property
    @abstractmethod
    def lowcmd_msgs_type(self):
        pass

    @property
    def n_dof(self) -> int:
        return len(self.joint_names)

    @property
    def lateral_friction(self):
        """Effective foot/ground lateral friction to enforce in the simulator.

        None -> keep the simulator defaults. PyBullet defaults every body to 0.5
        and combines friction multiplicatively at contact, so feet (0.5) on the
        ground plane (1.0) give an effective ~0.5 and the robot slips. Isaac Sim
        trains with terrain friction 1.0 (multiplicative combine) and a foot
        material centred on ~1.0, i.e. an effective ~1.0.
        """
        return None

    @property
    def foot_link_names(self) -> List[str]:
        """URDF link names of the feet that receive `lateral_friction`."""
        return []

    @property
    def mjcf_path(self) -> str:
        """Path to the robot's MuJoCo MJCF model (sim-to-sim). Override per robot."""
        raise NotImplementedError("No MuJoCo (MJCF) model configured for this robot.")

    @abstractmethod
    def foot_force_to_val(self, force):
        pass


class G1Configuration(RobotConfigurationAbstract):
    def __init__(self, dof: int = 27):
        # dof selects the URDF variant only. The joint list / n_dof stay 29 in
        # BOTH cases: the unitree LowCmd is always laid out on the 29-DOF canonical
        # order (waist_roll=13, waist_pitch=14), and the BulletWrapper maps the two
        # waist joints to None when they are `fixed` in the URDF.
        #   - 27 (mode 6): g1.urdf, waist_roll/pitch FIXED -> held rigid (commands
        #     for idx 13/14 arrive with kp=0, so a revolute waist would go limp).
        #   - 29 (mode 5): g1_29dof.urdf, waist_roll/pitch REVOLUTE -> actuated.
        if dof not in (27, 29):
            raise ValueError(f"G1 dof must be 27 or 29, got {dof}")
        self._dof = dof

        joint_name_unitree_order = [
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
            "waist_yaw_joint",
            "waist_roll_joint",
            "waist_pitch_joint",
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint",
            "left_wrist_yaw_joint",
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint",
        ]

        q_start = [0.0, 0.0, 0.641, 0.0, 0.0, 0.0, 1.0] + [0.0] * 29
        q_start[7 + joint_name_unitree_order.index("left_hip_pitch_joint")] = -0.5
        q_start[7 + joint_name_unitree_order.index("right_hip_pitch_joint")] = -0.5
        q_start[7 + joint_name_unitree_order.index("left_knee_joint")] = 1.0
        q_start[7 + joint_name_unitree_order.index("right_knee_joint")] = 1.0
        q_start[7 + joint_name_unitree_order.index("left_ankle_pitch_joint")] = -0.5
        q_start[7 + joint_name_unitree_order.index("right_ankle_pitch_joint")] = -0.5

        self._q_start = q_start
        self._joint_names = joint_name_unitree_order

    @property
    def high_level_period(self) -> float:
        return 1.0 / 1000.0

    @property
    def low_level_sub_step(self) -> int:
        return 6

    @property
    def urdf_path(self) -> str:
        # 29-DOF uses a sibling URDF (waist_roll/pitch revolute). Built from the
        # installed model dir so no new path constant / rebuild of path.py is needed.
        if self._dof == 29:
            return os.path.join(G1_DESCRIPTION_MODEL_DIR, "g1_29dof.urdf")
        return G1_DESCRIPTION_URDF_PATH

    @property
    def mjcf_path(self) -> str:
        # MuJoCo sim-to-sim scene, dof-switched like `urdf_path`. We load holosoma's
        # own WBT physics scene (robot + plane + Newton solver, collisions enabled) —
        # NOT the holosoma_retargeting model, whose geoms are contype=0 (visual only)
        # so the robot has no foot/ground contact and falls. Located via WBT_G1_MJCF_DIR
        # (exported by deploy.py for `--simulator mujoco`) so the colcon-installed
        # package needs no copy of the meshes.
        model_dir = os.environ.get("WBT_G1_MJCF_DIR")
        if not model_dir:
            raise RuntimeError(
                "WBT_G1_MJCF_DIR is not set. deploy.py exports it for `--simulator mujoco`; "
                "point it at the scenes dir holding scene_g1_27dof_wbt_plane.xml / "
                "scene_g1_29dof_wbt_plane.xml."
            )
        fname = "scene_g1_29dof_wbt_plane.xml" if self._dof == 29 else "scene_g1_27dof_wbt_plane.xml"
        return os.path.join(model_dir, fname)

    @property
    def q_start(self) -> List[float]:
        return self._q_start

    @property
    def joint_names(self) -> List[str]:
        return self._joint_names

    @property
    def feet_sensors_names(self) -> List[str]:
        return []

    @property
    def lateral_friction(self):
        # Match Isaac Sim's effective foot/ground friction (~1.0). Without this the
        # feet keep PyBullet's 0.5 default and the robot slips.
        return 1.0

    @property
    def foot_link_names(self) -> List[str]:
        return ["left_ankle_roll_link", "right_ankle_roll_link"]

    @property
    def lowstate_msgs_type(self):
        return LowStateG1

    @property
    def lowcmd_msgs_type(self):
        return LowCmdG1

    def foot_force_to_val(self, force):
        return force


class Go2Configuration(RobotConfigurationAbstract):
    def __init__(self):
        joint_name_unitree_order = [
            "FR_hip_joint",
            "FR_thigh_joint",
            "FR_calf_joint",
            "FL_hip_joint",
            "FL_thigh_joint",
            "FL_calf_joint",
            "RR_hip_joint",
            "RR_thigh_joint",
            "RR_calf_joint",
            "RL_hip_joint",
            "RL_thigh_joint",
            "RL_calf_joint",
        ]

        self._q_start = [0, 0, 0.25, 0, 0, 0, 1] + [0.0, 1.0, -2.0] * 4
        self._joint_names = joint_name_unitree_order

    @property
    def high_level_period(self) -> float:
        return 1.0 / 500.0

    @property
    def low_level_sub_step(self) -> int:
        return 12

    @property
    def urdf_path(self) -> str:
        return GO2_DESCRIPTION_URDF_PATH

    @property
    def q_start(self) -> List[float]:
        return self._q_start

    @property
    def joint_names(self) -> List[str]:
        return self._joint_names

    @property
    def feet_sensors_names(self) -> List[str]:
        return ["FR_foot", "FL_foot", "RR_foot", "RL_foot"]

    @property
    def lowstate_msgs_type(self):
        return LowStateGo2

    @property
    def lowcmd_msgs_type(self):
        return LowCmdGo2

    def foot_force_to_val(self, force):
        ## See https://github.com/inria-paris-robotics-lab/unitree_simulation/issues/6
        return 14.2 + 0.562 * force
