import rclpy
from rclpy.node import Node
import torch
from ament_index_python.packages import get_package_share_directory
import os
from robot_interface.msg import RobotState, RobotCMD
from sensor_msgs.msg import Joy
import time
from enum import Enum

class command:
    vx = 0.
    vy = 0.
    dyaw = 0.

class obs_scales:
    lin_vel = 2.0
    ang_vel = 0.25
    dof_pos = 1.0
    dof_vel = 0.05

class STATE(Enum):
    PASSIVE = 0
    STAND = 1
    SIT = 2
    RL = 3
    DAMPING = 4
    NOCMD = 5

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('RL_controller')

        # 获取包资源路径
        package_share_dir = get_package_share_directory('RL_controller')
        # 构建模型绝对路径
        model_path = os.path.join(package_share_dir, 'policy.jit')
        self.policy = self.load_JIT_model(model_path)

        self.init_params()
        self.counter = 0
        self.sub_joy = self.create_subscription(
                                Joy,
                                'joy',
                                self.joy_callback,
                                10)
        
        self.sub_robotState = self.create_subscription(
                                    RobotState,
                                    'ROS2_Robot_State',
                                    self.ctrl_callback,
                                    10)
        
        self.pub_robotCmd = self.create_publisher(
                                        RobotCMD,
                                        'RobotCMD',
                                        10)

    def init_params(self):

        self.default_pose = torch.tensor([0.0, 0.7, -1.5, # LF
                                          0.0, -0.7, 1.5, # LH
                                          0.0, 0.7, -1.5, # RF
                                          0.0, -0.7, 1.5]) # RH
        
        # [ 0.0415,  0.8655, -1.4859, -0.0068, -0.7869,  1.4371, -0.1045,  0.7009,
        # -1.4760, -0.0791, -0.7018,  1.3825]

        self.kps = torch.tensor([40, 40, 40,
                                 40, 40, 40,
                                 40, 40, 40,
                                 40, 40, 40])
        
        self.kds = torch.tensor([2, 2, 2,
                                 2, 2, 2,
                                 2, 2, 2,
                                 2, 2, 2])
        
        self.stand_kps = self.kps * 3
        self.damping_kds = self.kds * 3

        self.cmd = command()
        self.dof_nums = 12
        self.action_scale = 0.5
        self.action = torch.zeros(self.dof_nums)
        self.last_act = torch.zeros(self.dof_nums)

        self.obs_scales = obs_scales()
        self.commands_scale = torch.tensor([self.obs_scales.lin_vel, self.obs_scales.lin_vel, self.obs_scales.ang_vel])

        self.last_time = 0
        self.state = STATE.PASSIVE
        self.goto = STATE.NOCMD
        self.coff = 0.0

    def ctrl_callback(self, msg):

        if(self.counter % 10 == 0): # ROS2_Robot_State has 500 hz, policy run at 50 hz

            print("Interval: ", time.time()-self.last_time)
            self.last_time = time.time()

            self.robot_state = msg
            
            self.get_obs()
            self.get_cmd()
            self.pub_cmd()

        self.counter += 1
    
    def get_obs(self):
                    
        self.base_ang_vel = torch.tensor(self.robot_state.gyro)
        self.projected_gravity = self.get_gravity(torch.tensor(self.robot_state.quat))
        self.commands = torch.tensor([self.cmd.vx, self.cmd.vy, self.cmd.dyaw])
        self.dof_pos = torch.tensor(self.robot_state.q[:self.dof_nums])
        self.dof_vel = torch.tensor(self.robot_state.qd[:self.dof_nums])

        # tweak order
        new_order = [
            3, 4, 5, # LF
            9, 10, 11, # LH
            0, 1, 2, # RF
            6, 7, 8 # RH
        ]
        self.dof_pos = self.dof_pos[new_order]
        self.dof_vel = self.dof_vel[new_order]

    def run_policy(self):

        obs = torch.cat(( self.base_ang_vel * self.obs_scales.ang_vel, # 3dim
                          self.projected_gravity, # 3dim
                          self.commands * self.commands_scale, # 3dim
                          (self.dof_pos - self.default_pose) * self.obs_scales.dof_pos, # 12dim
                          self.dof_vel * self.obs_scales.dof_vel, # 12dim
                          self.last_act)).to(dtype=torch.float32) # 12dim

        action = self.policy(obs.unsqueeze(0)).squeeze()
        self.last_act = action
        self.motor_target = action * self.action_scale + self.default_pose

    def get_cmd(self):
        if(self.state == STATE.PASSIVE):
            self.motor_target = self.dof_pos
            if(self.goto == STATE.STAND):
                self.state = STATE.STAND
                self.goto == STATE.NOCMD
                self.start_pos = self.dof_pos
                self.sit_pos = self.dof_pos
                self.coff = 0.0
                print("Goto STAND!")

        elif(self.state == STATE.STAND):
            if(self.coff < 1.0):
                self.coff += 1.0 / 100
                if(self.coff >= 1.0):
                    self.coff = 1.0
                    print("STAND finished!")
                self.motor_target = (1.0 - self.coff) * self.start_pos +  self.coff * self.default_pose

            else: # already STAND
                self.motor_target = self.default_pose
                if(self.goto == STATE.RL):
                    self.state = STATE.RL
                    self.goto = STATE.NOCMD
                    print("Goto RL!")
                elif(self.goto == STATE.SIT):
                    self.state = STATE.SIT
                    self.goto = STATE.NOCMD
                    self.now_pos = self.dof_pos
                    self.coff = 0.0
                    print("Goto SIT!")

        elif(self.state == STATE.SIT):
            if(self.coff < 1.0):
                self.coff += 1.0 / 100
                if(self.coff >= 1.0):
                    self.coff = 1.0
                    print("SIT finished!")
                self.motor_target = (1.0 - self.coff) * self.now_pos + self.coff * self.sit_pos
            else: # already SIT
                self.motor_target = self.sit_pos
                if(self.goto == STATE.STAND):
                    self.state = STATE.STAND
                    self.goto = STATE.NOCMD
                    self.start_pos = self.dof_pos
                    self.coff = 0.0
                    print("Goto STAND!")

        elif(self.state == STATE.RL):
            self.run_policy()
            if(self.goto == STATE.SIT):
                self.state = STATE.SIT
                self.goto = STATE.NOCMD
                self.now_pos = self.dof_pos
                self.coff = 0.0
                print("Goto SIT!")
        
        elif(self.state == STATE.DAMPING):
            self.motor_target = self.dof_pos
            if(self.goto == STATE.STAND):
                self.state = STATE.STAND
                self.goto == STATE.NOCMD
                self.start_pos = self.dof_pos
                self.coff = 0.0
                print("Goto STAND!")


    def pub_cmd(self):

        # tweak order
        new_order = [ 6, 7, 8,    # RF
                        0, 1, 2,    # LF
                        9, 10, 11,  # RH
                        3, 4, 5]    # LH
        print("self.state: ", self.state)
        print("self.coff: ", self.coff)
        self.motor_target = self.motor_target[new_order].tolist()
        
        # publish robotCmd
        robotCmd = RobotCMD()
        robotCmd.q = [0.0] * 18
        robotCmd.qd = [0.0] * 18
        robotCmd.kp = [0.0] * 18
        robotCmd.kd = [0.0] * 18
        robotCmd.tau_ff = [0.0] * 18

        for i in range(self.dof_nums):
            robotCmd.q[i] = self.motor_target[i]
            robotCmd.kp[i] = self.kps[i]
            robotCmd.kd[i] = self.kds[i]
            if(self.state == STATE.STAND or self.state == STATE.SIT):
                robotCmd.kp[i] = self.stand_kps[i]
            elif(self.state == STATE.DAMPING):
                robotCmd.kp[i] = 0.0
                robotCmd.kd[i] = self.damping_kds[i]

        if(self.state == STATE.PASSIVE):
            robotCmd.kp = [0.0] * 18
            robotCmd.kd = [0.0] * 18

        self.pub_robotCmd.publish(robotCmd)


    def joy_callback(self, msg):

        self.cmd.vx = msg.axes[1]
        self.cmd.vy = msg.axes[0]
        self.cmd.dyaw = msg.axes[2]

        # print("vx:", self.commands.vx, "vy:", self.commands.vy, "dyaw:", self.commands.dyaw)
        self.goto = STATE.NOCMD
        if(msg.buttons[10]): # RB
            if(msg.buttons[3]): # Y
                self.goto = STATE.STAND
            elif(msg.buttons[0]): # A
                self.goto = STATE.SIT
            elif(msg.buttons[2]): # X
                self.goto = STATE.RL
            elif(msg.buttons[1]): # B
                self.state = STATE.DAMPING


    def load_JIT_model(self, model_path):
        # 加载 TorchScript 模型
        model = torch.jit.load(model_path)
        model.eval()
        
        return model

    def get_gravity(self, q):
        """
        input:
            q: (quat) w,x,y,z
        """
        v = torch.tensor([0.0, 0.0, -1.0], dtype=q.dtype) # gravity_vec
        q_w = q[0]
        q_vec = q[1:]
        a = v * (2.0 * q_w ** 2 - 1.0)
        b = torch.cross(q_vec, v) * q_w * 2.0
        c = q_vec * (2.0 * torch.dot(q_vec, v))

        return a - b + c


def main(args=None):

    rclpy.init(args=args)
    
    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()