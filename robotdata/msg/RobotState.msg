# 整个机器人状态，包含 12 个电机状态和 IMU 数据
robotdata/MotorState[12] motors
robotdata/IMU imu
int16 state    # -1-init, 0-pd_stand, 1-pd_mode, 2- RL mode
