import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
np.set_printoptions(precision=3, suppress=True)

# def forward_kinematics(self, theta1, theta2, theta3):

#         # T_0_1 (base_link to leg_front_r_1)
#         T_0_1 = translation(0.07500, -0.0445, 0) @ rotation_x(1.57080) @ rotation_z(theta1)

#         # T_1_2 (leg_front_r_1 to leg_front_r_2)
#         ## TODO: Implement the transformation matrix from leg_front_r_1 to leg_front_r_2
#         T_1_2 = translation(0, 0, 0.039) @ rotation_y(-1.57080) @ rotation_z(theta2)

#         # T_2_3 (leg_front_r_2 to leg_front_r_3)
#         ## TODO: Implement the transformation matrix from leg_front_r_2 to leg_front_r_3
#         T_2_3 = translation(0, -0.0494, 0.0685) @ rotation_y(1.57080) @ rotation_z(theta3)

#         # T_3_ee (leg_front_r_3 to end-effector)
#         T_3_ee = translation(0.06231, -0.06216, 0.018)

#         # TODO: Compute the final transformation. T_0_ee is a concatenation of the previous transformation matrices
#         T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee

#         # TODO: Extract the end-effector position. The end effector position is a 3x3 matrix (not in homogenous coordinates)
#         end_effector_position = T_0_ee[:3, 3]

#         return end_effector_position

def rotation_x(angle):
    ################################################################################################
    # TODO: [already done] paste lab 2 forward kinematics here
    ################################################################################################
    return np.array([
                [1, 0, 0, 0],
                [0, np.cos(angle), -np.sin(angle), 0],
                [0, np.sin(angle), np.cos(angle), 0],
                [0, 0, 0, 1]
            ])

def rotation_y(angle):
    ################################################################################################
    # TODO: [already done] paste lab 2 forward kinematics here
    ################################################################################################
    return np.array([
                [np.cos(angle), 0, np.sin(angle), 0],
                [0, 1, 0, 0],
                [-np.sin(angle), 0, np.cos(angle), 0],
                [0, 0, 0, 1]
            ])

def rotation_z(angle):
    ################################################################################################
    # TODO: [already done] paste lab 2 forward kinematics here
    ################################################################################################
    return np.array([
                [np.cos(angle), -np.sin(angle), 0, 0],
                [np.sin(angle), np.cos(angle), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])

def translation(x, y, z):
    ################################################################################################
    # TODO: [already done] paste lab 2 forward kinematics here
    ################################################################################################
    return np.array([
                [1, 0, 0, x],
                [0, 1, 0, y],
                [0, 0, 1, z],
                [0, 0, 0, 1]
            ])

class InverseKinematics(Node):

    def __init__(self):
        super().__init__('inverse_kinematics')
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.joint_subscription  # prevent unused variable warning

        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_command_controller/commands',
            10
        )

        self.joint_positions = None
        self.joint_velocities = None
        self.target_joint_positions = None
        self.counter = 0

        # Trotting gate positions, already implemented
        touch_down_position = np.array([0.05, 0.0, -0.14])
        stand_position_1 = np.array([0.025, 0.0, -0.14])
        stand_position_2 = np.array([0.0, 0.0, -0.14])
        stand_position_3 = np.array([-0.025, 0.0, -0.14])
        liftoff_position = np.array([-0.05, 0.0, -0.14])
        mid_swing_position = np.array([0.0, 0.0, -0.05])
        
        """
        np.array([
            [0.05, 0.0, -0.12],  # Touchdown
            [-0.05, 0.0, -0.12], # Liftoff
            [0.0, 0.0, -0.06]    # Mid-swing
        ])
        """

        ## trotting
        # TODO: Implement each leg’s trajectory in the trotting gait.
        rf_ee_offset = np.array([0.06, -0.09, 0])
        rf_ee_triangle_positions = np.array([
            touch_down_position, 
            stand_position_1, 
            stand_position_2, 
            stand_position_3, 
            liftoff_position, 
            mid_swing_position
        ]) + rf_ee_offset
        
        lf_ee_offset = np.array([0.06, 0.09, 0])
        lf_ee_triangle_positions = np.array([
            stand_position_3, 
            liftoff_position, 
            mid_swing_position, 
            touch_down_position,
            stand_position_1, 
            stand_position_2, 
        ]) + lf_ee_offset
        
        rb_ee_offset = np.array([-0.11, -0.09, 0])
        rb_ee_triangle_positions = np.array([
            stand_position_3, 
            liftoff_position, 
            mid_swing_position, 
            touch_down_position,
            stand_position_1, 
            stand_position_2, 
        ]) + rb_ee_offset
        
        lb_ee_offset = np.array([-0.11, 0.09, 0])
        lb_ee_triangle_positions = np.array([
            touch_down_position, 
            stand_position_1, 
            stand_position_2, 
            stand_position_3, 
            liftoff_position, 
            mid_swing_position
        ]) + lb_ee_offset


        self.ee_triangle_positions = [rf_ee_triangle_positions, lf_ee_triangle_positions, rb_ee_triangle_positions, lb_ee_triangle_positions]
        self.fk_functions = [self.fr_leg_fk, self.fl_leg_fk, self.br_leg_fk, self.lb_leg_fk]

        self.target_joint_positions_cache, self.target_ee_cache = self.cache_target_joint_positions()
        print(f'shape of target_joint_positions_cache: {self.target_joint_positions_cache.shape}')
        print(f'shape of target_ee_cache: {self.target_ee_cache.shape}')


        self.pd_timer_period = 1.0 / 200  # 200 Hz
        self.ik_timer_period = 1.0 / 100   # 10 Hz
        self.pd_timer = self.create_timer(self.pd_timer_period, self.pd_timer_callback)
        self.ik_timer = self.create_timer(self.ik_timer_period, self.ik_timer_callback)


    def fr_leg_fk(self, theta):
        # Already implemented in Lab 2
        T_RF_0_1 = translation(0.07500, -0.0445, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
        T_RF_1_2 = translation(0,0,0.039) @ rotation_y(-1.57080) @ rotation_z(theta[1])
        T_RF_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
        T_RF_3_ee = translation(0.06231, -0.06216, 0.01800)
        T_RF_0_ee = T_RF_0_1 @ T_RF_1_2 @ T_RF_2_3 @ T_RF_3_ee
        return T_RF_0_ee[:3, 3]

    def fl_leg_fk(self, theta):
        ################################################################################################
        # TODO: implement forward kinematics here
        ################################################################################################
        # Already implemented in Lab 2
        T_RF_0_1 = translation(0.07500, 0.0445, 0) @ rotation_x(-1.57080) @ rotation_z(theta[0])
        T_RF_1_2 = translation(0,0,0.039) @ rotation_y(-1.57080) @ rotation_z(theta[1])
        T_RF_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
        T_RF_3_ee = translation(0.06231, -0.06216, 0.01800)
        T_RF_0_ee = T_RF_0_1 @ T_RF_1_2 @ T_RF_2_3 @ T_RF_3_ee
        return T_RF_0_ee[:3, 3]

    def br_leg_fk(self, theta):
        ################################################################################################
        # TODO: implement forward kinematics here
        ################################################################################################
        
        # Already implemented in Lab 2
        T_RF_0_1 = translation(-0.07500, -0.0335, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
        T_RF_1_2 = translation(0,0,0.039) @ rotation_y(-1.57080) @ rotation_z(theta[1])
        T_RF_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
        T_RF_3_ee = translation(0.06231, -0.06216, 0.01800)
        T_RF_0_ee = T_RF_0_1 @ T_RF_1_2 @ T_RF_2_3 @ T_RF_3_ee
        return T_RF_0_ee[:3, 3]

    def lb_leg_fk(self, theta):
        ################################################################################################
        # TODO: implement forward kinematics here
        ################################################################################################
        # Already implemented in Lab 2
        T_RF_0_1 = translation(-0.07500, 0.0335, 0) @ rotation_x(-1.57080) @ rotation_z(theta[0])
        T_RF_1_2 = translation(0,0,0.039) @ rotation_y(-1.57080) @ rotation_z(theta[1])
        T_RF_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
        T_RF_3_ee = translation(0.06231, -0.06216, 0.01800)
        T_RF_0_ee = T_RF_0_1 @ T_RF_1_2 @ T_RF_2_3 @ T_RF_3_ee
        return T_RF_0_ee[:3, 3]

    def forward_kinematics(self, theta):
        return np.concatenate([self.fk_functions[i](theta[3*i: 3*i+3]) for i in range(4)])

    def listener_callback(self, msg):
        joints_of_interest = [
            'leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3', 
            'leg_front_l_1', 'leg_front_l_2', 'leg_front_l_3', 
            'leg_back_r_1', 'leg_back_r_2', 'leg_back_r_3', 
            'leg_back_l_1', 'leg_back_l_2', 'leg_back_l_3'
        ]
        self.joint_positions = np.array([msg.position[msg.name.index(joint)] for joint in joints_of_interest])
        self.joint_velocities = np.array([msg.velocity[msg.name.index(joint)] for joint in joints_of_interest])

    def inverse_kinematics_single_leg(self, target_ee, leg_index, initial_guess=[0.0, 0.0, 0.0]):
        leg_forward_kinematics = self.fk_functions[leg_index]

        def cost_function(theta):
            current_position = leg_forward_kinematics(theta)
            ################################################################################################
            # TODO: [already done] paste lab 3 inverse kinematics here
            ################################################################################################
            assert(len(theta)==3)
            cost = current_position - target_ee
            return cost, np.linalg.norm(cost, ord=1) ** 2

        def gradient(theta, epsilon=1e-3):
            grad = np.zeros(3)
            ################################################################################################
            # TODO: [already done] paste lab 3 inverse kinematics here
            ################################################################################################
            dfdtheta1 = (cost_function(theta + [epsilon, 0, 0])[1] 
                         - cost_function(theta + [-epsilon, 0, 0])[1]) / (2 * epsilon)
            dfdtheta2 = (cost_function(theta + [0, epsilon, 0])[1] 
                         - cost_function(theta + [0, -epsilon, 0])[1]) / (2 * epsilon)
            dfdtheta3 = (cost_function(theta + [0, 0, epsilon])[1]
                         - cost_function(theta + [0, 0, -epsilon])[1]) / (2 * epsilon)
            return np.array([dfdtheta1, dfdtheta2, dfdtheta3])

        theta = np.array(initial_guess, dtype=np.float64)
        learning_rate = 10.0 # Set the learning rate
        max_iterations = 100 # Set the maximum number of iterations
        tolerance = 0.00005 # Set the tolerance for the L1 norm of the error

        cost_l = []
        for _ in range(max_iterations):
            ################################################################################################
            # TODO: [already done] paste lab 3 inverse kinematics here
            ################################################################################################
            grad = gradient(theta)
            theta -= learning_rate * grad
            cost, l1 = cost_function(theta)
            # cost_l.append(cost)
            if l1.mean() < tolerance:
                print("converged")
                break
            if _ == max_iterations:
                print("max iteractions reached")

        return theta

    def interpolate_leg(self, triangle_positions, t):
        target = 0
        if t % 6.0 < 1:
            target = triangle_positions[0] + (t % 1.0) * (triangle_positions[1] - triangle_positions[0])
        elif t % 6.0 < 2:
            target = triangle_positions[1] + (t % 1.0) * (triangle_positions[2] - triangle_positions[1])
        elif t % 6.0 < 3:
            target = triangle_positions[2] + (t % 1.0) * (triangle_positions[3] - triangle_positions[2])
        elif t % 6.0 < 4:
            target = triangle_positions[3] + (t % 1.0) * (triangle_positions[4] - triangle_positions[3])
        elif t % 6.0 < 5:
            target = triangle_positions[4] + (t % 1.0) * (triangle_positions[5] - triangle_positions[4])
        elif t % 6.0 < 6:
            target = triangle_positions[5] + (t % 1.0) * (triangle_positions[0] - triangle_positions[5])
        return target
    
    def interpolate_triangle(self, t, leg_index):
        return self.interpolate_leg(self.ee_triangle_positions[leg_index], t)

    def cache_target_joint_positions(self):
        # Calculate and store the target joint positions for a cycle and all 4 legs
        target_joint_positions_cache = []
        target_ee_cache = []
        for leg_index in range(4):
            target_joint_positions_cache.append([])
            target_ee_cache.append([])
            target_joint_positions = [0] * 3
            for t in np.arange(0, 1, 0.02):
                print(t)
                target_ee = self.interpolate_triangle(t, leg_index)
                target_joint_positions = self.inverse_kinematics_single_leg(target_ee, leg_index, initial_guess=target_joint_positions)

                target_joint_positions_cache[leg_index].append(target_joint_positions)
                target_ee_cache[leg_index].append(target_ee)

        # (4, 50, 3) -> (50, 12)
        target_joint_positions_cache = np.concatenate(target_joint_positions_cache, axis=1)
        target_ee_cache = np.concatenate(target_ee_cache, axis=1)
        
        return target_joint_positions_cache, target_ee_cache

    def get_target_joint_positions(self):
        target_joint_positions = self.target_joint_positions_cache[self.counter]
        target_ee = self.target_ee_cache[self.counter]
        self.counter += 1
        if self.counter >= self.target_joint_positions_cache.shape[0]:
            self.counter = 0
        return target_ee, target_joint_positions

    def ik_timer_callback(self):
        if self.joint_positions is not None:
            target_ee, self.target_joint_positions = self.get_target_joint_positions()
            current_ee = self.forward_kinematics(self.joint_positions)

            self.get_logger().info(
                f'Target EE: {target_ee}, \
                Current EE: {current_ee}, \
                Target Angles: {self.target_joint_positions}, \
                Target Angles to EE: {self.forward_kinematics(self.target_joint_positions)}, \
                Current Angles: {self.joint_positions}')

    def pd_timer_callback(self):
        if self.target_joint_positions is not None:
            command_msg = Float64MultiArray()
            command_msg.data = self.target_joint_positions.tolist()
            self.command_publisher.publish(command_msg)

def main():
    rclpy.init()
    inverse_kinematics = InverseKinematics()
    
    try:
        rclpy.spin(inverse_kinematics)
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        # Send zero torques
        zero_torques = Float64MultiArray()
        zero_torques.data = [0.0] * 12
        inverse_kinematics.command_publisher.publish(zero_torques)
        
        inverse_kinematics.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
