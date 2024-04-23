
import os
import inspect

# currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
# parentdir = os.path.dirname(currentdir)
# os.sys.path.insert(0, parentdir)
import torch
import time
import numpy as np
from omegaconf import OmegaConf
from hydra import compose, initialize
from hydra.utils import to_absolute_path
# from MPC_Controller.Parameters import Parameters
# from MPC_Controller.utils import DTYPE
# from MPC_Controller.common.StateEstimator import StateEstimate

from utils.reformat import omegaconf_to_dict, print_dict

from rl_games.algos_torch.model_builder import ModelBuilder
from rl_games.algos_torch.running_mean_std import RunningMeanStd
from rl_games.algos_torch import torch_ext

## OmegaConf & Hydra Config
OmegaConf.register_new_resolver('eq', lambda x, y: x.lower()==y.lower())
OmegaConf.register_new_resolver('contains', lambda x, y: x.lower() in y.lower())
OmegaConf.register_new_resolver('if', lambda pred, a, b: a if pred else b)
OmegaConf.register_new_resolver('resolve_default', lambda default, arg: default if arg=='' else arg)

class WeightPolicy:
    def __init__(self, 
                 task="Aliengo", 
                 checkpoint="../Aliengo.pth",
                 num_envs=1):
        print('______class weight policy________')
        self.num_actions = 12
        self.num_obs = 48
        self.device = "cpu" 
        self.is_determenistic = True

        # hydra global initialization
        initialize(config_path="../cfg")
        cfg = compose(config_name="config", 
                      overrides=["checkpoint="+checkpoint, 
                                 "task="+task, 
                                 "num_envs="+str(num_envs)])

        self.lin_vel_scale = cfg["task"]["env"]["learn"]["linearVelocityScale"]
        self.ang_vel_scale = cfg["task"]["env"]["learn"]["angularVelocityScale"]
        self.dof_pos_scale = cfg["task"]["env"]["learn"]["dofPositionScale"]
        self.dof_vel_scale = cfg["task"]["env"]["learn"]["dofVelocityScale"]
        cfg_dict = omegaconf_to_dict(cfg)
        print_dict(cfg_dict)

        # ensure checkpoints can be specified as relative paths
        if cfg.checkpoint:
            cfg.checkpoint = to_absolute_path(cfg.checkpoint)

        rlg_config_dict = omegaconf_to_dict(cfg.train)
        
        # prepare config and params dict
        params = rlg_config_dict['params']
        config = params['config']
        model_builder = ModelBuilder()
        config['network'] = model_builder.load(params)
    
        print('Found checkpoint:')
        print(params['load_path'])
        load_path = params['load_path']
        num_agents = config['num_actors']

        obs_shape = (self.num_obs,)

        self.clip_actions = config.get('clip_actions', True)
        self.normalize_input = config['normalize_input']

        # use model directly
        state_dict_ckpt = torch_ext.load_checkpoint(load_path)
        # load model
        self.model = config['network'].build({
                'actions_num' : self.num_actions,
                'input_shape' : obs_shape,
                'num_seqs' : num_agents
            })
        self.model.to(self.device)
        self.model.eval()
        self.model.load_state_dict(state_dict_ckpt['model'])
        # load obs normalizer
        if self.normalize_input:
            self.running_mean_std = RunningMeanStd(obs_shape).to(self.device)
            self.running_mean_std.eval()
            self.running_mean_std.load_state_dict(state_dict_ckpt['running_mean_std'])

        self.num_agents = num_agents
        self.obs = torch.ones([self.num_agents, self.num_obs], 
                              requires_grad=False, dtype=torch.float, device=self.device)

    def _rescale_actions(self, low, high, action):
        d = (high - low) / 2.0
        m = (high + low) / 2.0
        scaled_action =  action * d + m
        return scaled_action