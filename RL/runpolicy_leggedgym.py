import os
import isaacgym
from legged_gym.envs import *
from legged_gym.utils import  get_args, task_registry
import lcm
import torch
from obslcm import observ_t
from obslcm import action_t
import numpy as np
import time
from legged_gym.envs.base.legged_robot import *
from legged_gym.envs.base.base_task import *

device = 'cpu'
msg_action = action_t() 
msg_obslist = []
cheeta_action = []
arg = get_args()

def run_policy(args,*obs):
    global arg,env
    print('task:',args.task)
    env_cfg, train_cfg = task_registry.get_cfgs(name='iust')
    # override some parameters for testing
    env_cfg.terrain.num_rows = 5
    env_cfg.terrain.num_cols = 5
    env_cfg.terrain.curriculum = False
    env_cfg.noise.add_noise = False
    env_cfg.domain_rand.randomize_friction = False
    env_cfg.domain_rand.push_robots = False

    env, _ = task_registry.make_env(name='iust', args=args, env_cfg=env_cfg)
    # obs = env.get_observations()
    obs = torch.tensor([obs], requires_grad=False, dtype = torch.float, device = device)

    train_cfg.runner.resume = True
    ppo_runner, train_cfg = task_registry.make_alg_runner(env=env, name='iust', args=args, train_cfg=train_cfg)
    policy = ppo_runner.get_inference_policy(device=env.device)
    
    while True:
        t_p = time.time()
        actions = policy(obs.detach())
        # print(np.shape(actions))
        print('actions:', actions)
        print('--------------------------------------------------------')
        newobs, _, rews, dones, info = env.step(actions[0].detach())
        print('--------------------------------------------------------')
        print('newobs:',newobs)
        print('reward:',rews)
        # time.sleep(2)
        print('data rate: {:.5f}'.format(time.time()- t_p))
        print('#####################################################################################')
    
    # return actions
    

def my_handler(channel, data):
    global obs, msg_obslist, device
    msg_obs = observ_t.decode(data)
    # t_p = time.time()
    print("Received message on channel \"%s\"" % channel)
    msg_obslist [:3] = msg_obs.base_lin_vel
    msg_obslist [3:6] = msg_obs.base_ang_vel
    msg_obslist [6:9] = msg_obs.gravity
    msg_obslist [9:12] = msg_obs.commands
    msg_obslist [12:24] = msg_obs.dof_pos
    msg_obslist [24:36] = msg_obs.dof_vel
    msg_obslist [36:48] = msg_obs.action 
    obs = msg_obslist
    # obs = torch.tensor([msg_obslist], requires_grad=False, dtype = torch.float, device = device)
    print('--------------------------------------------------------------------------------')
    print('observations:',obs)
    print('--------------------------------------------------------------------------------')
    # actions= run_policy(arg,obs)
    run_policy(arg,obs)
    # msg_obslist = []
    # # actions_tau = LeggedRobot._compute_torques(action)
    # msg_action.tau = actions[0][0]
    # # print('Transfer Time: {:.5f}'.format(time.time()- t_p))
    # lc.publish("ACTION", msg_action.encode())
    # # time.sleep(1000)
    # print('actions tau*:', actions)
    # print('################################################################################')

msg_obs = observ_t()                                  
lc = lcm.LCM() # "udpm://224.0.55.55:5001?ttl=225"
subscription = lc.subscribe("OBSERVATION", my_handler)


if __name__ == '__main__':
    lc.handle()
    while True:
        args = get_args()
        run_policy(args)
