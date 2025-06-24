# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class SiriusFlatCfg( LeggedRobotCfg ):
    class env( LeggedRobotCfg.env ):
        num_envs = 4096
        num_actions = 12
        num_observations = 45

    class terrain( LeggedRobotCfg.terrain ):
        mesh_type = 'plane'
        measure_heights = False

    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.445] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            "LF_HAA": 0.0,
            "LH_HAA": 0.0,
            "RF_HAA": 0.0,
            "RH_HAA": 0.0,

            "LF_HFE": 0.8,
            "LH_HFE": -0.8,
            "RF_HFE": 0.8,
            "RH_HFE": -0.8,

            "LF_KFE": -1.6,
            "LH_KFE": 1.6,
            "RF_KFE": -1.6,
            "RH_KFE": 1.6,
        }
            
    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        stiffness = {'HAA': 40., 'HFE': 40., 'KFE': 40.}  # [N*m/rad]
        damping = {'HAA': 2., 'HFE': 2., 'KFE': 2.}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.5
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 5
        use_actuator_network = False #True
        # actuator_net_file = "{LEGGED_GYM_ROOT_DIR}/resources/actuator_nets/anydrive_v3_lstm.pt"

    class asset( LeggedRobotCfg.asset ):
        file = "{LEGGED_GYM_ROOT_DIR}/resources/robots/sirius_diff_release/urdf/sirius_diff_new.urdf"
        name = "sirius"
        foot_name = "FOOT"
        penalize_contacts_on = ["calf", "thigh"]
        terminate_after_contacts_on = ["base"]
        self_collisions = 1 #1 to disable, 0 to enable...bitwise filter
        flip_visual_attachments = False

    class commands( LeggedRobotCfg.commands ):
        heading_command = False
        resampling_time = 4.
        class ranges( LeggedRobotCfg.commands.ranges ):
            ang_vel_yaw = [-1.5, 1.5]

    class domain_rand( LeggedRobotCfg.domain_rand):
        randomize_base_mass = True
        added_mass_range = [-5., 5.]
        friction_range = [0., 1.5] # on ground planes the friction combination mode is averaging, i.e total friction = (foot_friction + 1.)/2.
  
    class rewards( LeggedRobotCfg.rewards ):
        base_height_target = 0.445
        max_contact_force = 350
        only_positive_rewards = True
        soft_dof_vel_limit = 0.8
        class scales( LeggedRobotCfg.rewards.scales ):
            tracking_lin_vel = 1.0
            tracking_ang_vel = 0.5
            orientation = -5.0
            feet_air_time = 3.
            base_height = -200.
            posture = 1.0
            # dof_vel = -0.005
            # dof_vel_limits = -0.1
    
    class noise( LeggedRobotCfg.noise ):
        add_noise = True
        noise_level = 1.0 # scales other values
        class noise_scales( LeggedRobotCfg.noise.noise_scales ):
            dof_pos = 0.03
            dof_vel = 1.5
            lin_vel = 0.1
            ang_vel = 0.5
            gravity = 0.05
            height_measurements = 0.1

    class sim ( LeggedRobotCfg.sim ):
        dt =  0.004

class SiriusFlatCfgPPO( LeggedRobotCfgPPO ):
    class policy( LeggedRobotCfgPPO.policy ):
        actor_hidden_dims = [128, 64, 32]
        critic_hidden_dims = [128, 64, 32]
        activation = 'elu' # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid

    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = "sirius_diff_release"
        load_run = -1
        max_iterations = 1200
