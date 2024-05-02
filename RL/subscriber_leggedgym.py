import os
import isaacgym
import lcm
import torch
from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg
from obslcm import observ_t 
from obslcm import action_t
from policy_leggedgym import *
import numpy as np
import time
from legged_gym.envs import *
# 48 obs and 12 action / leggedgym

# default_joint_angles = { # = target angles [rad] when action = 0.0
#             'FL_hip_joint': 0.1,   # [rad]  
#             'RL_hip_joint': 0.1,   # [rad]
#             'FR_hip_joint': -0.1 ,  # [rad]
#             'RR_hip_joint': -0.1,   # [rad]

#             'FL_thigh_joint': 0.8,     # [rad]
#             'RL_thigh_joint': 1.,   # [rad]
#             'FR_thigh_joint': 0.8,     # [rad]
#             'RR_thigh_joint': 1.,   # [rad]

#             'FL_calf_joint': -1.5,   # [rad]
#             'RL_calf_joint': -1.5,    # [rad]
#             'FR_calf_joint': -1.5,  # [rad]
#             'RR_calf_joint': -1.5,    # [rad]
#         }
# ([ FL*.hip, FL.thigh, FL.calf, FR*.hip, ..., RL*.hip, ..., RR*.hip, ...])

device = 'cpu'
cfg = LeggedRobotCfg

def quat_rotate_inverse(q, v):
    shape = q.shape
    q_w = q[:, -1]
    q_vec = q[:, :3]
    a = v * (2.0 * q_w ** 2 - 1.0).unsqueeze(-1)
    b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
    c = q_vec * \
        torch.bmm(q_vec.view(shape[0], 1, 3), v.view(
            shape[0], 3, 1)).squeeze(-1) * 2.0
    return a - b + c

def steps(actions,obs):
        clip_actions = cfg.normalization.clip_actions
        actions = torch.clip(actions, -clip_actions, clip_actions).to(device)
        torques = torch.zeros(cfg.env.num_envs, cfg.env.num_actions, dtype=torch.float, device=device, requires_grad=False)
        ## step physics and render each frame
        for _ in range(cfg.control.decimation):
            torques = compute_torques(actions).view(torques.shape)


        ## return clipped obs, clipped states (None), rewards, dones and infos
        clip_obs = cfg.normalization.clip_observations
        obs = torch.clip(obs, -clip_obs, clip_obs)
        return torques.numpy() 

        # action_scale = 0.25 
        # actions = actions * action_scale
        # return actions

def compute_torques(actions):
    global default_dof_pos
    default_dof_pos = np.array([0,0.8,-1.5,0,0.8,-1.5,0,0.8,-1.5,0,0.8,-1.5])
    torque_limits = 20
    action_scale = 0.25
    p_gains = 10
    d_gains = 0
    actions_scaled = actions * action_scale
    control_type = "P"
    if control_type=="P":
        torques = p_gains*(actions_scaled + default_dof_pos - dof_pos) - d_gains*dof_vel
    # elif control_type=="V":
    #     torques = p_gains*(actions_scaled - dof_vel) - d_gains*(dof_vel - last_dof_vel)/sim_params.dt
    # elif control_type=="T":
    #     torques = actions_scaled
    return torch.clip(torques, -torque_limits, torque_limits)


# ***PUBLISH ACTION***
msg_action = action_t() 
msg_obslist = [] 
cheeta_action = []
def my_handler(channel, data):
    global obs, msg_obslist, device, dof_pos, dof_vel
    msg_obs = observ_t.decode(data)
    t_p = time.time()
    print("Received message on channel \"%s\"" % channel)
    # print(" base_lin_vel = %s" % str(msg_obs.base_lin_vel))

    v = np.array(msg_obs.base_lin_vel)
    v = v.reshape((1,3))
    ang_vel = np.array(msg_obs.base_ang_vel)
    ang_vel = ang_vel.reshape((1,3))
    quat = np.array(msg_obs.quaternion)
    quat = quat.reshape((1,4))

    v = torch.tensor(v)
    ang_vel = torch.tensor(ang_vel)
    quat = torch.tensor(quat)



    msg_obslist [:3] = np.array(quat_rotate_inverse(quat,v))
    msg_action [3:6] = np.array(quat_rotate_inverse(quat,ang_vel))

    msg_obslist [:3] = np.array(msg_obslist [:3]) * 2.0 
    msg_obslist [3:6] = np.array(msg_action [3:6]) * 0.25

    msg_obslist [6:9] = np.array(msg_obs.gravity)
    msg_obslist [9:12] = np.array(msg_obs.commands) * [2.0, 2.0, 0.25]
   
    msg_obslist [12:15] = np.array(msg_obs.dof_pos [3:6]) * np.array([-1,-1,-1])
    msg_obslist [15:18] = np.array(msg_obs.dof_pos [:3]) * np.array([1,-1,-1])
    msg_obslist [18:21] = np.array(msg_obs.dof_pos [9:12]) * np.array([-1,-1,-1])
    msg_obslist [21:24] = np.array(msg_obs.dof_pos [6:9]) * np.array([1,-1,-1])
    dof_pos = np.array(msg_obslist[12:24])

    msg_obslist [24:27] = np.array(msg_obs.dof_vel [3:6]) * 0.05
    msg_obslist [27:30] = np.array(msg_obs.dof_vel [:3]) * 0.05
    msg_obslist [30:33] = np.array(msg_obs.dof_vel [9:12]) * 0.05
    msg_obslist [33:36] = np.array(msg_obs.dof_vel [6:9]) * 0.05
    dof_vel = np.array(msg_obslist[24:36])

    msg_obslist [36:39] = np.array(msg_obs.action [3:6])
    msg_obslist [39:42] = np.array(msg_obs.action [:3])
    msg_obslist [42:45] = np.array(msg_obs.action [9:12])
    msg_obslist [45:48] = np.array(msg_obs.action [6:9])
    # print(" observation = %s" % str(msg_obslist))
    # print('--------------------------------------------------------------------------------')
    # msg_obsarray = np.concatenate(msg_obslist).ravel() # shape (48,)
    obs = torch.tensor([msg_obslist], requires_grad=False, dtype = torch.float, device = device)
    print('--------------------------------------------------------------------------------')
    print('observations:',obs)
    print('--------------------------------------------------------------------------------')
    
    # env, _ = task_registry.make_env(name='iust')
    actions = Policy(obs.detach()) # shape (12,)
    actions = torch.tensor(np.array(actions)).detach()
    actions = steps(actions,obs)

    cheeta_action [:3] = actions [0][3:6] * np.array([-1,-1,-1])
    cheeta_action [3:6] = actions [0][:3] * np.array([1,-1,-1])
    cheeta_action [6:9] = actions [0][9:12] * np.array([-1,-1,-1])
    cheeta_action [9:12] = actions [0][6:9] * np.array([1,-1,-1])
    # cheeta_action = [-10,0,0,0,0,0,0,0,0,0,0,0]
    # cheeta_action =[ 0.0469,  -1.3995,  -5.0445, -0.5454,  -1.7124,  -4.4102, 0.8912, 0.1605,  -5.2695,0.1275,  -0.7016,
        #   -6.8975]
     
    
    msg_obslist = []
    msg_action.tau = cheeta_action
    print('Transfer Time: {:.5f}'.format(time.time()- t_p))
    time.sleep(20000)
    lc.publish("ACTION", msg_action.encode())
    print('actions:', cheeta_action)
    print('################################################################################')
    

# ***SUBSCRIBE OBSERVATION***
msg_obs = observ_t()                                  
lc = lcm.LCM("udpm://224.0.55.55:5001?ttl=225") # "udpm://224.0.55.55:5001?ttl=225"
subscription = lc.subscribe("OBSERVATION", my_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass
