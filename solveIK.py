#!/usr/bin/env python
import os
import sys
import numpy as np
from solution.solveFK import FK

class IK:

    # JOINT LIMITS
    lower = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upper = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    fk = FK()

    def __init__(self):
        pass

    @staticmethod
    def calcJacobian(q):
        """
        Calculate the Jacobian of the end effector in a given configuration.
        INPUT:
        q - 1 x 7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]
        OUTPUT:
        J - the Jacobian matrix 
        """
        # YOUR CODE STARTS HERE
        J = np.zeros((6, 7))
        T_accumulated = np.eye(4)
        jointPositions, T0e = IK.fk.forward(q)
        # First, use the FK formula you have written to calculate the end position and orientation T0e, and extract the end position.
        p_e = T0e[:3, 3]

        # Traverse 7 joints and extract the Z-axis and origin position of each joint coordinate system
        for i in range(7):
            # Obtain the Z-axis of the current joint coordinate system (the third column of the transformation matrix)
            z_axis = T_accumulated[:3, 2]
            # Obtain the position of the origin of the current joint coordinate system (the fourth column of the transformation matrix)
            p_joint = T_accumulated[:3, 3]
            
            # Calculate the i-th column of the Jacobian matrix
            J[:3, i] = np.cross(z_axis, (p_e - p_joint))  # Linear velocity section
            J[3:, i] = z_axis                             # Angular velocity part
            
            # Update the cumulative transformation matrix to prepare for the next joint.
            a, alpha, d = IK.fk.dh_params[i]
            theta = q[i]
            Ti = IK.fk.build_dh_transform(a, alpha, d, theta)
            T_accumulated = T_accumulated @ Ti

        # YOUR CODE ENDS HERE
        return J

    @staticmethod
    def cal_target_transform_vec(target, current):
        """
        Calculate the displacement vector and axis of rotation from 
        the current frame to the target frame

        INPUTS:
        target - 4x4 numpy array representing the desired transformation from
                 end effector to world

        current - 4x4 numpy array representing the current transformation from
                  end effector to world

        OUTPUTS:
        translate_vec - a 3-element numpy array containing the target translation vector from
                        the current frame to the target frame, expressed in the world frame

        rotate_vec - a 3-element numpy array containing the target rotation vector from
                     the current frame to the end effector frame
        """

        translate_vec = []
        rotate_vec = []
        # YOUR CODE STARTS HERE
        #Calculate the translational error: Target position [x, y, z] - Current position [x, y, z]
        translate_vec = target[:3, 3] - current[:3, 3]

        # 2. Calculate rotation error: Use the X, Y, and Z axis vectors of the two sets of rotation matrices to calculate the cross-product difference
        rotate_vec = 0.5 * (np.cross(current[:3, 0], target[:3, 0]) + 
                            np.cross(current[:3, 1], target[:3, 1]) + 
                            np.cross(current[:3, 2], target[:3, 2]))
        # YOUR CODE ENDS HERE
        return translate_vec, rotate_vec

    def check_joint_constraints(self,q,target):
        """
        Check if the given candidate solution respects the joint limits.

        INPUTS
        q - the given solution (joint angles)

        target - 4x4 numpy array representing the desired transformation from
                 end effector to world

        OUTPUTS:
        success - True if some predefined certain conditions are met. Otherwise False
        """

        success = False

        # YOUR CODE STARTS HERE
        # 1. Check if any joint is beyond the limit (any joint being below the lower limit or above the upper limit)
        if np.any(q < IK.lower) or np.any(q > IK.upper):
            return False
            
        # 2. Check whether the end error is sufficiently small
        _, current_T = IK.fk.forward(q)
        trans_err, rot_err = IK.cal_target_transform_vec(target, current_T)
        
        # Sum the absolute values of the translational and rotational errors to obtain the total error.
        total_error = np.linalg.norm(trans_err) + np.linalg.norm(rot_err)
        
        # Set a precision threshold (for example, 0.001)
        if total_error < 1e-3:
            return True
            
        return False
     
        # YOUR CODE ENDS HERE

        return success


    @staticmethod
    def solve_ik(q,target):
        """
        Uses the method you prefer to calculate the joint velocity 

        INPUTS:
        q - the current joint configuration, a "best guess" so far for the final solution

        target - a 4x4 numpy array containing the target end effector pose

        OUTPUTS:
        dq - a desired joint velocity
        Note: Make sure that it will smoothly decay to zero magnitude as the task is achieved.
        """

        dq = []
        # YOUR CODE STARTS HERE
        # 1. Obtain the current pose and calculate the error
        _, current_T = IK.fk.forward(q)
        trans_err, rot_err = IK.cal_target_transform_vec(target, current_T)
        
        # Combine the translational and rotational errors into a 6-dimensional error vector dx.
        dx = np.concatenate((trans_err, rot_err))
        
        # 2. Calculate the Jacobian matrix
        J = IK.calcJacobian(q)

        J_pinv = np.linalg.pinv(J, rcond=1e-2)#Increase the rcond tolerance to prevent the pseudo-inverse matrix from exploding near the singular point.
        
        # 3. Calculate the pseudo-inverse of the Jacobian matrix
        J_pinv = np.linalg.pinv(J)
        
        # 4. Calculate the joint velocity dq (multiply by a small gain gain to help stabilize and converge)
        gain = 0.5 
        dq = gain * (J_pinv @ dx)
        #Enforce a maximum step size limit! If the calculated speed is too high, reduce it proportionally.
        max_step = 0.1  # 
        if np.linalg.norm(dq) > max_step:
            dq = (dq / np.linalg.norm(dq)) * max_step
        # YOUR CODE ENDS HERE

        return dq

    def inverse(self, target, initial_guess):
        """
        Solve the inverse kinematics of the robot arm

        INPUTS:
        target - 4x4 numpy array representing the desired transformation from
        end effector to world

        initial_guess - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6], which
        is the "initial guess" from which to proceed with the solution process (has set up for you)

        OUTPUTS:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6], giving the
        solution if success is True or the closest guess if success is False.

        success - True if IK is successfully solved. Otherwise False
        """
        

        # YOUR CODE STARTS HERE
        q = np.copy(initial_guess)
        success = False
        
        max_iter = 2000  # Set the maximum number of iterations
        q_history = []
        
        for i in range(max_iter):
            q_history.append(np.copy(q))
            #Store the current joint angle in the historical record.
            # 1. Check whether the current q meets the requirements (with extremely small error and not exceeding the limit)
            if self.check_joint_constraints(q, target):
                success = True
                break
                
            # 2.Calculate the amount of quantity that needs to be updated, dq
            dq = IK.solve_ik(q, target)
            
            # 3. Update joint angles
            q = q + dq
            
            # 4. Force the value of q to be within the physically permitted upper and lower limits.
            q = np.clip(q, IK.lower, IK.upper)
            
        # If the loop ends without a break, it indicates that the convergence has not occurred within the maximum number of iterations.
        if not success:
            q_history.append(np.copy(q))
        # YOUR CODE ENDS HERE

        return q_history, success

if __name__ == "__main__":
    pass
