from legged_gym.envs import *
from legged_gym.utils import  get_args, task_registry
import lcm
from dofstatelcm import dofstate_t
import numpy as np
from legged_gym.envs.base.legged_robot import *

msg_dofstate = dofstate_t()
cheeta_dofstate = []
args = get_args()
lc = lcm.LCM()

def run_policy(args):
    env_cfg, train_cfg = task_registry.get_cfgs(name='iust')
    ## override some parameters for testing
    env_cfg.terrain.num_rows = 5
    env_cfg.terrain.num_cols = 5
    env_cfg.terrain.curriculum = False
    env_cfg.noise.add_noise = False
    env_cfg.domain_rand.randomize_friction = False
    env_cfg.domain_rand.push_robots = False

    env, _ = task_registry.make_env(name='iust', args=args, env_cfg=env_cfg)
    obs = env.get_observations()

    train_cfg.runner.resume = True
    ppo_runner, train_cfg = task_registry.make_alg_runner(env=env, name='iust', args=args, train_cfg=train_cfg)
    policy = ppo_runner.get_inference_policy(device=env.device)
    while True:
        actions = policy(obs.detach())
        obs, _, rews, dones, infos, torque = env.step(actions.detach())
        print('observations:' ,obs)
        print('tau:',torque)
        print('---------------------------------------------------------------------')

        # cheeta_dofstate [:3] = obs [0][15:18] 
        # cheeta_dofstate [3:6] = obs [0][12:15] 
        # cheeta_dofstate [6:9] = obs [0][21:24] 
        # cheeta_dofstate [9:12] = obs [0][18:21] 

        # cheeta_dofstate [12:15] = obs [0][27:30] 
        # cheeta_dofstate [15:18] = obs [0][24:27] 
        # cheeta_dofstate [18:21] = obs [0][33:36] 
        # cheeta_dofstate [21:24] = obs [0][30:33] 

        # msg_dofstate.dof_pos = cheeta_dofstate [:12]
        # msg_dofstate.dof_vel = cheeta_dofstate [12:24]
        # lc.publish("DOFSTATE", msg_dofstate.encode())
        # pass
    

if __name__ == '__main__':
    while True:
        args = get_args()
        run_policy(args)
