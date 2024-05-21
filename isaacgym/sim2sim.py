import lcm
import torch
import mujoco
import mujoco_viewer
import numpy as np
from tqdm import tqdm
from Config import Config
from base.SimBase import SimBase
from scipy.spatial.transform import Rotation as R


def quaternion_to_euler_array(quat):
    # Ensure quaternion is in the correct format [x, y, z, w]
    x, y, z, w = quat
    # Roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)
    # Pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch_y = np.arcsin(t2)
    # Yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)
    # Returns roll, pitch, yaw in a NumPy array in radians
    return np.array([roll_x, pitch_y])  # , yaw_z


def pd_control(target_q, q, kp, target_dq, dq, kd):
    return (target_q + [0, 0, 0.5, -0.8, -0.3, 0, 0, 0.5, -0.8, -0.3] - q) * kp + (target_dq - dq) * kd


class Sim2Sim(SimBase):
    def __init__(self, _lc, _cfg, _policy):
        super().__init__(_lc, _cfg, _policy)
        self.model = mujoco.MjModel.from_xml_path(self.cfg.sim_config.mujoco_model_path)
        self.model.opt.timestep = self.cfg.control.dt
        self.data = mujoco.MjData(self.model)
        mujoco.mj_step(self.model, self.data)
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)

    def get_obs(self):
        q = self.data.qpos.astype(np.double)
        dq = self.data.qvel.astype(np.double)
        quat = self.data.sensor('orientation').data[[1, 2, 3, 0]].astype(np.double)
        r = R.from_quat(quat)
        v = r.apply(self.data.qvel[:3], inverse=True).astype(np.double)  # In the base frame
        omega = self.data.sensor('angular-velocity').data.astype(np.double)
        gvec = r.apply(np.array([0., 0., -1.]), inverse=True).astype(np.double)
        return q, dq, quat, v, omega, gvec

    def run(self):
        cnt_pd_loop = 0
        for _ in tqdm(range(int(self.cfg.env.run_duration / self.cfg.control.dt)), desc="Simulating..."):
            # Obtain an observation
            q, dq, quat, v, omega, gvec = self.get_obs()
            q = q[-self.cfg.env.num_actions:]
            dq = dq[-self.cfg.env.num_actions:]
            # 1000hz -> 100hz
            if cnt_pd_loop % self.cfg.control.decimation == 0:
                eu_ang = quaternion_to_euler_array(quat)
                self.get_action(q, dq, eu_ang, omega, cnt_pd_loop)
                # publish pd ctrl paras
                _cmd = [0x89] * self.cfg.env.num_actions
                self.set_real_target(_cmd, self.target_q, self.target_dq)

            # Generate PD control
            tau = pd_control(self.target_q, q, self.cfg.robot_config.kps,
                             self.target_dq, dq, self.cfg.robot_config.kds)  # Calc torques
            tau = np.clip(tau, -self.cfg.robot_config.tau_limit, self.cfg.robot_config.tau_limit)  # Clamp torques
            self.data.ctrl = tau
            mujoco.mj_step(self.model, self.data)
            self.viewer.render()
            cnt_pd_loop += 1

        self.viewer.close()


if __name__ == '__main__':
    mode_path = "policies/policy_1.pt"
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
    policy = torch.jit.load(mode_path)
    mybot = Sim2Sim(lc, Config, policy)
    mybot.run()
