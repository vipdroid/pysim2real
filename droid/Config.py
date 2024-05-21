import numpy as np


class Config:
    """
    Configuration class for the X02 humanoid robot.
    """
    class env:
        # change the observation dim
        frame_stack = 15
        num_single_obs = 40
        num_observations = int(frame_stack * num_single_obs)
        num_actions = 10
        run_duration = 30.0  # 单位s

    class cmd:
        vx = 0.5
        vy = 0.0
        yaw = 0.0

    class sim_config:
        mujoco_model_path = f'../robots/X02Lite/X02Lite.xml'

    class control:
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.35
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 10  # 100hz(10ms)
        dt = 0.001  # 1ms
        period = 0.01  # 10ms
        cycle_time = 0.5  # refer period

    class normalization:
        class obs_scales:
            lin_vel = 2.
            ang_vel = 1.
            dof_pos = 1.
            dof_vel = 0.05

        clip_observations = 18.

    class robot_config:
        clip_actions_upper = np.array([1.05,  0.35,  1.92,  0.08,  1.3,  1.05,  0.35,  1.92,  0.08,  1.3], dtype=np.double)
        clip_actions_lower = np.array([-1.05, -0.35, -0.52, -1.92, -1.3, -1.05, -0.35, -0.52, -1.92, -1.3], dtype=np.double)
        kps = np.array([400, 500, 250, 350, 30, 400, 500, 250, 350, 30], dtype=np.double)
        kds = np.array([2, 2, 2, 2, 1, 2, 2, 2, 2, 1], dtype=np.double)
        tau_limit = np.array([50, 300, 72, 150, 45, 50, 300, 72, 150, 45], dtype=np.double)  # 峰值扭矩
        # tau_limit = np.array([20, 150, 40,  60, 25, 20, 150, 40,  60, 25], dtype=np.double)  # 额定扭矩

