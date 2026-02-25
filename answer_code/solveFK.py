#!/usr/bin/env python
import numpy as np
class FK():

    def __init__(self):
        # Define geometric parameters for computing the forward kinematics. 
        # The required parameters are provided in the assignment description document.
        self.dh_params = self.init_dh_params()
        self.joint_offsets = self.init_joint_offsets()

    def init_dh_params(self):
        """
        Initialize dh parameters from all intermediate frames in the form [a, alpha, d]
        (refer to assignment description)
        """

        dh_params = [
	    [0,      -np.pi/2, 0.333],
            [0,       np.pi/2, 0],
            [0.082,   np.pi/2, 0.316],
            [-0.082, -np.pi/2, 0],
            [0,       np.pi/2, 0.384],
            [0.088,   np.pi/2, 0],
            [0,       0,       0.21]
	]
        return dh_params

    def init_joint_offsets(self):
        """
        Initialize joint position offsets
        relative to intermediate frames defined using
        DH conventions 
        (refer to assignment description)
        """

        joint_offsets = [
	    [0, 0, 0.141],
            [0, 0, 0],
            [0, 0, 0.195],
            [0, 0, 0],
            [0, 0, 0.125],
            [0, 0, 0],
            [0, 0, 0.051]
	]
        return joint_offsets

    def build_dh_transform(self, a, alpha, d, theta):
        """
        Construct transformation matrix T,
        using DH parameters and conventions
        """
        # YOUR CODE STARTS HERE
        T = [
	    [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),               np.cos(alpha),                d],
            [0,              0,                           0,                            1]
	]
        # YOUR CODE ENDS HERE
        return T

    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions - 7 x 3 matrix, where each row corresponds to a rotational joint of the robot
                         Each row contains the [x,y,z] coordinates in the world frame of the respective 
                         joint's center in meters. The base of the robot is located at [0,0,0].

        T0e - a homogeneous transformation matrix,
              representing the end effector frame expressed in the world frame
        """
        jointPositions = []
        # YOUR CODE STARTS HERE
        T0e = np.eye(4)
        for i in range(7):
		# 1. Extract the DH parameters and angles of the current joint
                a, alpha, d = self.dh_params[i]
                theta = q[i]
            
		# 2. Calculate the local transformation matrix of the current joint and multiply it to the global transformation matrix T0e
                Ti = self.build_dh_transform(a, alpha, d, theta)
                T0e = T0e @ Ti
            
 		# 3. Calculate the visual center position of the joint (taking into account the offset)
                offset = self.joint_offsets[i]
 		# Construct the local offset vector [x, y, z, 1]
                local_offset = np.array([offset[0], offset[1], offset[2], 1.0])
            
 		# Convert the local offset to the global world coordinate system
                global_pos = T0e @ local_offset
            
		# Extract the first three elements [x, y, z] and add them to the list
                jointPositions.append(global_pos[:3])
        jointPositions = np.array(jointPositions)
        # YOUR CODE ENDS HERE

        T0e = np.matmul(T0e, self.build_dh_transform(0, 0, 0, -np.pi/4))
        return jointPositions, T0e

if __name__ == "__main__":
    pass
