#!/usr/bin/env python

# Note: Complete the following subfunctions to generate valid transformation matrices 
# from a translation vector and Euler angles, or a sequence of 
# successive rotations around z, y, and x.
import numpy as np

class transformation():

    @staticmethod
    def trans(d):
        """
        Calculate pure translation homogenous transformation by d
        """
        # YOUR CODE STARTS HERE
        T = np.eye(4)  # Create a 4x4 identity matrix
        T[0,3] = d[0]  #x
        T[1, 3] = d[1] #y
        T[2, 3] = d[2] #z
        return T
        # YOUR CODE ENDS HERE
    @staticmethod
    def roll(a):
        """
        Calculate homogenous transformation for rotation around x axis by angle a
        """
        # YOUR CODE STARTS HERE
        T = np.eye(4)
        T[1, 1] = np.cos(a)
        T[1, 2] = -np.sin(a)
        T[2, 1] = np.sin(a)
        T[2, 2] = np.cos(a)
        return T
        # YOUR CODE ENDS HERE

    @staticmethod
    def pitch(a):
        """
        Calculate homogenous transformation for rotation around y axis by angle a
        """
        # YOUR CODE STARTS HERE
        T = np.eye(4)
        T[0, 0] = np.cos(a)
        T[0, 2] = np.sin(a)
        T[2, 0] = -np.sin(a)
        T[2, 2] = np.cos(a)
        return T
        # YOUR CODE ENDS HERE

    @staticmethod
    def yaw(a):
        """
        Calculate homogenous transformation for rotation around z axis by angle a
        """
        # YOUR CODE STARTS HERE
        T = np.eye(4)
        T[0, 0] = np.cos(a)
        T[0, 1] = -np.sin(a)
        T[1, 0] = np.sin(a)
        T[1, 1] = np.cos(a)
        return T
        # YOUR CODE ENDS HERE

    @staticmethod
    def transform(d,rpy):
        """
        Calculate a homogenous transformation for translation by d and
        rotation corresponding to roll-pitch-yaw euler angles
        """
        # YOUR CODE STARTS HERE
	# rpy = [roll, pitch, yaw]
        T_trans = transformation.trans(d)
        T_roll = transformation.roll(rpy[0])
        T_pitch = transformation.pitch(rpy[1])
        T_yaw = transformation.yaw(rpy[2])
        
        # Matrix multiplication order: Translation * Yaw * Pitch * Roll
        return T_trans @ T_yaw @ T_pitch @ T_roll
        # YOUR CODE ENDS HERE
    
if __name__ == "__main__":
    pass
