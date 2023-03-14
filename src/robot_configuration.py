
import numpy as np

robot_params = {}

robot_params["ur5"] = {
        "dt":   0.001,
        "kp":   np.array([300, 300, 300, 30, 30, 1]),
        "kd":   np.array([20, 20, 20, 5, 5, 0.5]),
        "q_0":  np.array([-0.32, -0.78, -2.56, -1.63, -1.57, 3.49]),
        "joint_names": [
            "shoulder_pan_join",
            "shoulder_lift_joint"
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
            ],
        "ee_frame": "tool0",
        "control_mode": "point", # available: "trajectory", "point" 
        "real_robot": False,
        "control_type": "position",
        "gripper_sim": False,
        "soft_gripper": False,
        "spawn_x": 0.5,
        "spawn_y": 0.35,
        "spawn_z": 1.75,
        "buffer_size": 100
        }

# dt  = 0.001
# kp  = np.array([300, 300, 300, 30, 30, 1])
# kd  = np.array([20, 20, 20, 5, 5, 0.5])
# q_0 = np.array([-0.32, -0.78, -2.56, -1.63, -1.57, 3.49])
# joint_names = [
#         "shoulder_pan_join",
#         "shoulder_lift_joint",
#         "elbow_joint",
#         "wrist_1_joint",
#         "wrist_2_joint",
#         "wrist_3_joint"
#         ]
# ee_frame = "tool0"
# control_mode = "point" # available: trajectory, point 
# real_robot = False
# control_type = "position"
# gripper_sim = False
# soft_gripper = False
# spawn_x = 0.5
# spawn_y = 0.35
# spawn_z = 1.75
