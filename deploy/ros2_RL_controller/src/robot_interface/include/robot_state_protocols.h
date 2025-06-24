#ifndef ROBOT_STATE_PROTOCOLS_H
#define ROBOT_STATE_PROTOCOLS_H

struct Robot_Control_Motor_Cmd {
    double q[18];
    double qd[18];
    double kp[18];
    double kd[18];
    double tau_ff[18];
};

struct Robot_State{
    double quat[4];
    double gyro[3];
    double acc[3];
    double q[18];
    double qd[18];
};

struct Sim_Plot {
    double foot_pos_des_[4][3]{};
    double foot_pos_[4][3]{};
    double pos_des_[3]{};
    double pos_[3]{};
};
#endif