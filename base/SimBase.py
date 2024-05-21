import time
import math
import copy
import torch
import select
import threading
import numpy as np
from base import lcmCtrl
from collections import deque


class SimBase(object):
    def __init__(self, _lc, _cfg, _policy):
        self.run_thread = None
        self.run_flag = True
        self.lc = _lc
        self.cfg = _cfg
        self.policy = _policy
        self.rc_state_sub = self.lc.subscribe("rc_state", self._rc_state_cb)
        self.body_state_sub = self.lc.subscribe("body_state", self._body_state_cb)
        self.joint_state_sub = self.lc.subscribe("lower_joint_state", self._joint_state_cb)
        self.joint_target_pub = lcmCtrl.joint_target_t()
        self.joint_target_pub.period_ms = self.cfg.control.decimation  # 设置low_level消息更新周期，与策略网络推理周期保持一致
        # rc_state
        self.keys = np.zeros(4, dtype=np.byte)
        self.rocker = np.zeros(4, dtype=np.float32)
        # body state
        self.rpy = np.zeros(3, dtype=np.float32)
        self.aBody = np.zeros(3, dtype=np.float32)
        self.omegaBody = np.zeros(3, dtype=np.float32)
        self.quat = np.zeros(4, dtype=np.float32)
        # joint state
        self.aec = np.zeros(self.cfg.env.num_actions, dtype=np.float32)
        self.pc = np.zeros(self.cfg.env.num_actions, dtype=np.float32)
        self.vc = np.zeros(self.cfg.env.num_actions, dtype=np.float32)
        self.iqc = np.zeros(self.cfg.env.num_actions, dtype=np.float32)
        self.timestamp_ms = 0
        # joint target
        self.target_q = np.zeros(self.cfg.env.num_actions, dtype=np.double)
        self.target_dq = np.zeros(self.cfg.env.num_actions, dtype=np.double)
        self.action = np.zeros(self.cfg.env.num_actions, dtype=np.double)
        # obs
        self.hist_obs = deque()
        for _ in range(self.cfg.env.frame_stack):
            self.hist_obs.append(np.zeros([1, self.cfg.env.num_single_obs], dtype=np.double))

    def next_tic(self):
        # 1000hz(1ms) -> 100hz(10ms)
        while self.timestamp_ms == 0 and self.run_flag:
            time.sleep(0.0001)
        self.timestamp_ms = 0

    def get_obs(self):
        q = np.array(self.pc, dtype=np.float32)
        dq = np.array(self.vc, dtype=np.float32)
        eu_ang = np.array(self.rpy[:2], dtype=np.float32)
        omega = np.array(self.omegaBody, dtype=np.float32)
        return q, dq, eu_ang, omega

    def run(self):
        pass

    def get_action(self, q, dq, eu_ang, omega, cnt_pd_loop):
        q = q[-self.cfg.env.num_actions:]
        dq = dq[-self.cfg.env.num_actions:]
        eu_ang[eu_ang > math.pi] -= 2 * math.pi
        obs = np.zeros([1, self.cfg.env.num_single_obs], dtype=np.float32)
        obs[0, 0] = math.sin(2 * math.pi * cnt_pd_loop * self.cfg.control.dt / self.cfg.control.cycle_time)
        obs[0, 1] = math.cos(2 * math.pi * cnt_pd_loop * self.cfg.control.dt / self.cfg.control.cycle_time)
        obs[0, 2] = self.cfg.cmd.vx * self.cfg.normalization.obs_scales.lin_vel
        obs[0, 3] = self.cfg.cmd.vy * self.cfg.normalization.obs_scales.lin_vel
        obs[0, 4] = self.cfg.cmd.yaw * self.cfg.normalization.obs_scales.ang_vel
        obs[0, 5:15] = q * self.cfg.normalization.obs_scales.dof_pos
        obs[0, 15:25] = dq * self.cfg.normalization.obs_scales.dof_vel
        obs[0, 25:35] = self.action
        obs[0, 35:38] = omega
        obs[0, 38:40] = eu_ang
        obs = np.clip(obs, -self.cfg.normalization.clip_observations, self.cfg.normalization.clip_observations)
        self.hist_obs.append(obs)
        self.hist_obs.popleft()
        policy_input = np.zeros([1, self.cfg.env.num_observations], dtype=np.float32)
        for i in range(self.cfg.env.frame_stack):
            policy_input[0, i * self.cfg.env.num_single_obs: (i + 1) * self.cfg.env.num_single_obs] = self.hist_obs[i][0, :]
        self.action[:] = self.policy(torch.tensor(policy_input))[0].detach().numpy()
        self.action = np.clip(self.action, self.cfg.robot_config.clip_actions_lower,
                              self.cfg.robot_config.clip_actions_upper)
        self.target_q = self.action * self.cfg.control.action_scale

    def poll(self, cb=None):
        try:
            while self.run_flag:
                timeout = 0.005
                rfds, wfds, efds = select.select([self.lc.fileno()], [], [], timeout)
                if rfds:
                    self.lc.handle()
                    # print(f'Freq {1. / (time.time() - t)} Hz')
                else:
                    continue
        except KeyboardInterrupt:
            pass

    def spin(self):
        self.run_thread = threading.Thread(target=self.poll, daemon=False)
        self.run_thread.start()

    def init_robot(self, low=False):
        if low:
            final_goal = np.array([0.0, -0.0, 0.5, -0.8, -0.3,
                                   0.0, -0.0, 0.5, -0.8, -0.3])
        else:
            final_goal = np.zeros(self.cfg.env.num_actions)
        target_sequence = []
        # self.joint_target_pub.period_ms = 20
        self.next_tic()
        target = self.pc
        while np.max(np.abs(target - final_goal)) > 0.01:
            target -= np.clip((target - final_goal), -0.005, 0.005)
            target_sequence += [copy.deepcopy(target)]
        for target in target_sequence:
            next_target = target
            next_target = next_target
            # publish pd ctrl paras
            self.next_tic()
            _cmd = [0x89] * self.cfg.env.num_actions
            # _cmd = np.array([0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06], dtype=np.int16)
            self.set_real_target(_cmd, next_target, self.target_dq)
        while (self.keys[3] == 0) and (self.run_flag == True):
            self.next_tic()
            # final_goal[1] = self.rocker[0] * 0.5
            # final_goal[6] = self.rocker[2] * 0.5
            _cmd = [0x89] * self.cfg.env.num_actions
            # _cmd = np.array([0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06], dtype=np.int16)
            self.set_real_target(_cmd, final_goal, self.target_dq)
        # self.set_real_stop()

    def set_real_stop(self):
        for i in range(100):  # 1s stop
            self.next_tic()
            _cmd = [0x06] * self.cfg.env.num_actions
            _dq = [0x00] * self.cfg.env.num_actions
            self.set_real_target(_cmd, self.pc, _dq)
        self.run_flag = False

    def set_real_target(self, cmd, q, qd):
        self.joint_target_pub.cmd = cmd
        self.joint_target_pub.pt = q
        self.joint_target_pub.vt = qd
        self.joint_target_pub.iqt = [15] * self.cfg.env.num_actions
        self.joint_target_pub.kp = self.cfg.robot_config.kps
        self.joint_target_pub.kd = self.cfg.robot_config.kds
        self.lc.publish("lower_joint_target", self.joint_target_pub.encode())

    def _rc_state_cb(self, channel, data):
        msg = lcmCtrl.rc_state_t.decode(data)
        temp_keys = np.array(msg.keys)
        self.keys = temp_keys.astype(np.uint8)
        self.rocker = msg.rocker
        if self.keys[1] > 128 and self.keys[2] > 128:  # 急停
            self.run_flag = False
            print("Emergency stop")

    def _body_state_cb(self, channel, data):
        msg = lcmCtrl.body_state_t.decode(data)
        self.rpy = msg.rpy
        self.omegaBody = msg.omegaBody
        self.aBody = msg.aBody

    def _joint_state_cb(self, channel, data):
        msg = lcmCtrl.joint_state_t.decode(data)
        self.aec = msg.aec
        self.pc = msg.pc
        self.vc = msg.vc
        self.iqc = msg.iqc
        self.timestamp_ms = msg.timestamp_ms

