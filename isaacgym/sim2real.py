import lcm
import time
import torch
import numpy as np
from tqdm import tqdm
from base.SimBase import SimBase
from Config import Config


class Sim2Real(SimBase):
    def __init__(self, _lc, _cfg, _policy):
        super().__init__(_lc, _cfg, _policy)

    def run(self):
        cnt_pd_loop = 1.0
        for _ in tqdm(range(int(self.cfg.env.run_duration / self.cfg.control.period)), desc="x02 running..."):
            self.next_tic()
            cnt_pd_loop += self.cfg.control.decimation
            # Obtain an observation
            q, dq, eu_ang, omega = self.get_obs()
            self.get_action(q, dq, eu_ang, omega, cnt_pd_loop)
            # publish pd ctrl paras
            _cmd = [0x86] * self.cfg.env.num_actions
            self.target_q = self.target_q + [ 0, 0, 0.5, -0.8, -0.3, 0, 0, 0.5, -0.8, -0.3]
            self.set_real_target(_cmd, self.target_q, self.target_dq)
        self.set_real_stop()


if __name__ == '__main__':
    mode_path = "policies/policy_x02.pt"
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
    policy = torch.jit.load(mode_path)
    mybot = Sim2Real(lc, Config, policy)
    mybot.spin()
    mybot.init_robot(low=True)   # 屈膝状态
    # mybot.init_robot(low=False)  # 直立状
    time.sleep(1)
    mybot.run()
