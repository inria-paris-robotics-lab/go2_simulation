"""MuJoCo backend for sim-to-sim, mirroring bullet_wrapper.py.

Drop-in alternative to BulletWrapper behind AbstractSimulatorWrapper, so the rest
of the deployment stack (PD loop in simulation_node, the unitree DDS bridge, the
watchdog) is unchanged — only the physics engine differs. This is exactly what a
sim-to-sim test wants: the same control/observation path driven by a second
simulator.

step() honours the same contract as BulletWrapper.step():
    q = [base_pos(3), base_quat_xyzw(4), joint_pos(n)]
    v = [base_lin_vel(3, LOCAL frame), base_ang_vel(3, LOCAL frame), joint_vel(n)]
    a = finite-difference of v (base part in LOCAL frame)
    f = per-foot contact force (empty for G1)

The canonical joint list is the full 29-DOF unitree order in both the 27- and
29-DOF cases. Joints absent from the MJCF (waist_roll/pitch in g1_27dof.xml) map
to None, exactly like BulletWrapper maps `fixed` joints: their q/v read 0 and
their torque command is dropped.

Conventions cross-checked against holosoma's own MuJoCo simulator:
  - quaternion: MuJoCo is wxyz, the contract is xyzw.
  - base velocity: mj_objectVelocity(..., flg_local=1) returns [ang(3), lin(3)]
    already in the local base frame (no manual world->local rotation needed).
"""

import os

import numpy as np
import mujoco

from unitree_simulation.abstract_wrapper import AbstractSimulatorWrapper


def _quat_wxyz_to_xyzw(q):
    return np.array([q[1], q[2], q[3], q[0]])


def _quat_xyzw_to_wxyz(q):
    return np.array([q[3], q[0], q[1], q[2]])


class MujocoWrapper(AbstractSimulatorWrapper):
    def __init__(self, robot_config):
        mjcf_path = robot_config.mjcf_path
        if not mjcf_path or not os.path.exists(mjcf_path):
            raise FileNotFoundError(
                f"MuJoCo model not found: {mjcf_path!r}. deploy.py exports WBT_G1_MJCF_DIR "
                "for `--simulator mujoco`; set it to the dir holding g1_27dof.xml / g1_29dof.xml."
            )

        # Meshes are referenced relative to the MJCF (compiler meshdir="assets/"),
        # so loading by absolute path resolves them.
        self.model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.data = mujoco.MjData(self.model)

        # Match the high-level integration step (sim_dt = high_level_period / sub_step).
        self.dt = robot_config.sim_dt
        self.model.opt.timestep = self.dt

        # Foot/ground friction. MuJoCo already defaults slide friction to ~1.0 (unlike
        # PyBullet's 0.5), so this usually matches Isaac without help; we still pin the
        # ground geom for parity with BulletWrapper when the config asks for it.
        if robot_config.lateral_friction is not None:
            for gname in ("floor", "ground"):
                gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, gname)
                if gid != -1:
                    self.model.geom_friction[gid, 0] = robot_config.lateral_friction
                    break

        # Base free joint addressing.
        self.base_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        free_jnt = next(
            (i for i in range(self.model.njnt) if self.model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE),
            -1,
        )
        if free_jnt == -1 or self.base_body_id == -1:
            raise RuntimeError("MuJoCo model has no freejoint / no 'pelvis' body.")
        self.base_qpos_adr = self.model.jnt_qposadr[free_jnt]  # 7: [x y z qw qx qy qz]
        self.base_qvel_adr = self.model.jnt_dofadr[free_jnt]   # 6: [vx vy vz wx wy wz]

        # Per-joint addressing in the canonical 29-DOF unitree order; None where the
        # joint is absent from the model (waist_roll/pitch in the 27-DOF variant).
        self.joint_names = robot_config.joint_names
        self.joint_qpos_adr = []
        self.joint_qvel_adr = []
        for name in self.joint_names:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid == -1:
                self.joint_qpos_adr.append(None)
                self.joint_qvel_adr.append(None)
            else:
                self.joint_qpos_adr.append(self.model.jnt_qposadr[jid])
                self.joint_qvel_adr.append(self.model.jnt_dofadr[jid])

        self.q_start = robot_config.q_start
        self.n_feet = len(robot_config.feet_sensors_names)

        # Base lock ("robot hanged at startup"): pinned kinematically until unlock_base.
        self.locked = True
        self._locked_base_qpos = None

        # Finite differences for acceleration.
        self.v_last = None

        # Passive viewer (parity with BulletWrapper's GUI); degrade to headless gracefully.
        self.viewer = None
        if not os.environ.get("WBT_MUJOCO_HEADLESS"):
            try:
                from mujoco import viewer as mj_viewer

                self.viewer = mj_viewer.launch_passive(self.model, self.data)
            except Exception:
                self.viewer = None
        # Refresh the viewer at ~1 kHz, not at every sub-step (~6 kHz).
        self._view_decim = max(1, int(round(1.0 / (1000.0 * self.dt))))
        self._step_count = 0

        self.reset()

        # Startup diagnostic (shown in the sim tmux pane) — which model loaded and
        # whether the feet make ground contact at spawn.
        print(
            f"[MujocoWrapper] {os.path.basename(mjcf_path)} | nq={self.model.nq} nv={self.model.nv} "
            f"nu={self.model.nu} ngeom={self.model.ngeom} | spawn base_z="
            f"{float(self.data.qpos[self.base_qpos_adr + 2]):.3f} initial_contacts={self.data.ncon}",
            flush=True,
        )

    def _pin_base(self):
        self.data.qpos[self.base_qpos_adr : self.base_qpos_adr + 7] = self._locked_base_qpos
        self.data.qvel[self.base_qvel_adr : self.base_qvel_adr + 6] = 0.0

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)

        base = np.empty(7)
        base[0:3] = self.q_start[0:3]
        base[3:7] = _quat_xyzw_to_wxyz(self.q_start[3:7])
        self._locked_base_qpos = base.copy()
        self.data.qpos[self.base_qpos_adr : self.base_qpos_adr + 7] = base

        for i, adr in enumerate(self.joint_qpos_adr):
            if adr is not None:
                self.data.qpos[adr] = self.q_start[7 + i]

        self.data.qvel[:] = 0.0
        self.locked = True
        self.v_last = None
        mujoco.mj_forward(self.model, self.data)
        if self.viewer is not None:
            self.viewer.sync()

    def unlock_base(self):
        self.locked = False

    def step(self, tau_cmd):
        # Apply joint torques as generalized forces (the MJCF has no actuators).
        self.data.qfrc_applied[:] = 0.0
        for i, adr in enumerate(self.joint_qvel_adr):
            if adr is not None:
                self.data.qfrc_applied[adr] = tau_cmd[i]

        if self.locked:
            self._pin_base()

        mujoco.mj_step(self.model, self.data)

        if self.locked:
            # Re-pin the base and refresh derived quantities (xpos/cvel) for reading.
            self._pin_base()
            mujoco.mj_forward(self.model, self.data)

        # ── read state ─────────────────────────────────────────────────────────
        base_pos = np.array(self.data.qpos[self.base_qpos_adr : self.base_qpos_adr + 3])
        base_quat_xyzw = _quat_wxyz_to_xyzw(
            self.data.qpos[self.base_qpos_adr + 3 : self.base_qpos_adr + 7]
        )

        n = len(self.joint_names)
        joint_pos = np.zeros(n)
        joint_vel = np.zeros(n)
        for i in range(n):
            if self.joint_qpos_adr[i] is not None:
                joint_pos[i] = self.data.qpos[self.joint_qpos_adr[i]]
                joint_vel[i] = self.data.qvel[self.joint_qvel_adr[i]]

        # Base velocity in the LOCAL base frame: mj_objectVelocity -> [ang(3), lin(3)].
        vel6 = np.zeros(6)
        mujoco.mj_objectVelocity(
            self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.base_body_id, vel6, 1
        )
        base_ang_vel = vel6[0:3]
        base_lin_vel = vel6[3:6]

        q_current = np.concatenate((base_pos, base_quat_xyzw, joint_pos))
        v_current = np.concatenate((base_lin_vel, base_ang_vel, joint_vel))
        a_current = (
            (v_current - self.v_last) / self.dt if self.v_last is not None else np.zeros_like(v_current)
        )
        self.v_last = v_current
        f_current = np.zeros(self.n_feet)

        self._step_count += 1
        if self.viewer is not None and self._step_count % self._view_decim == 0:
            if self.viewer.is_running():
                self.viewer.sync()

        return q_current, v_current, a_current, f_current
