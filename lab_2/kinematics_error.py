import numpy as np

def forward_kinematics(theta1, theta2, theta3, error):

    def rotation_x(angle):
        # rotation about the x-axis implemented for you
        return np.array([
            [1, 0, 0, 0],
            [0, np.cos(angle), -np.sin(angle), 0],
            [0, np.sin(angle), np.cos(angle), 0],
            [0, 0, 0, 1]
        ])

    def rotation_y(angle):
        return np.array([
            [np.cos(angle), 0, np.sin(angle), 0],
            [0, 1, 0, 0],
            [-np.sin(angle), 0, np.cos(angle), 0],
            [0, 0, 0, 1]
        ]) 
        ## TODO: Implement the rotation matrix about the y-axis
        # return np.array([
        # ])
    
    def rotation_z(angle):
        return np.array([
            [np.cos(angle), -np.sin(angle), 0, 0],
            [np.sin(angle), np.cos(angle), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        ## TODO: Implement the rotation matrix about the z-axis
        # return np.array([
        # ])

    def translation(x, y, z):
        ## TODO: Implement the translation matrix
        # return np.array([
        # ])
        return np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        ])

    # T_0_1 (base_link to leg_front_r_1)
    T_0_1 = translation(0.07500, -0.0445, 0) @ rotation_x(1.57080) @ rotation_z(theta1)

    # T_1_2 (leg_front_r_1 to leg_front_r_2)
    ## TODO: Implement the transformation matrix from leg_front_r_1 to leg_front_r_2
    T_1_2 = translation(0, 0, 0.039) @ rotation_y(-1.57080) @ rotation_z(theta2)

    # T_2_3 (leg_front_r_2 to leg_front_r_3)
    ## TODO: Implement the transformation matrix from leg_front_r_2 to leg_front_r_3
    error_on_y = -error * np.sin(0.585)
    error_on_z = error * np.cos(0.585)
    T_2_3 = translation(0, -0.0494 + error_on_y, 0.0685 + error_on_z) @ rotation_y(1.57080) @ rotation_z(theta3)

    # T_3_ee (leg_front_r_3 to end-effector)
    T_3_ee = translation(0.06231, -0.06216, 0.018)

    # TODO: Compute the final transformation. T_0_ee is a concatenation of the previous transformation matrices
    T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee

    # TODO: Extract the end-effector position. The end effector position is 0.068a 3x3 matrix (not in homogenous coordinates)
    end_effector_position = T_0_ee[:3, 3]

    return end_effector_position



deg0_std = forward_kinematics(0, 0, 0, 0)
deg45_std = forward_kinematics(0, 0, np.pi / 4, 0)

error = 0.001
for i in range(3):
    error *= 2
    print("error =", error)
    deg0 = forward_kinematics(0, 0, 0, error)
    deg45 = forward_kinematics(0, 0, 45, error)
    print("0 degrees: ", deg0)
    print("difference: ", np.linalg.norm(deg0 - deg0_std))
    print("45 degrees: ", deg45)
    print("difference: ", np.linalg.norm(deg45 - deg45_std))