from legged_gym.envs import *
from legged_gym.utils import  get_args, task_registry
from pathlib import Path
import os
import copy
import torch

def export_model(args):

    # create env
    env_cfg, train_cfg = task_registry.get_cfgs(name=args.task)
    env, _ = task_registry.make_env(name=args.task, args=args, env_cfg=env_cfg)
    
    ppo_runner, train_cfg = task_registry.make_alg_runner(env=env, name=args.task, args=args, train_cfg=train_cfg)
    ppo_runner.load(ORIGINAL_MODEL_PATH)

    ppo_runner.alg.actor_critic.eval()

    path = os.path.join(SAVE_DIR, 'policy.jit')
    model = copy.deepcopy(ppo_runner.alg.actor_critic.actor).to('cpu')
    traced_script_module = torch.jit.script(model)
    traced_script_module.save(path)
    print('Exported policy as jit script!')


if __name__ == '__main__':

    args = get_args()
    ORIGINAL_MODEL_PATH = args.model_path
    SAVE_DIR = Path(ORIGINAL_MODEL_PATH).parent
    
    export_model(args)
