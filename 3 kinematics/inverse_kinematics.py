'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np
from autograd import grad
from scipy.optimize import fmin
from math import atan2


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def error_func(self, joint_angles,joint_names, target):
        Te= identity(4)
        for i in range(0,len(joint_angles)):
            Ts = self.local_trans(joint_names[i], joint_angles[i])
            Te= np.dot(Te,Ts)
        End = np.matrix(self.from_transform(Te))
        e = target -End
        return np.linalg.norm(e)

    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_names = self.chains[effector_name]
        joint_angles =[ self.perception.joint[joint_name] for joint_name in joint_names]
        #joint_angles = self.chains[effector_name]
        print(transform)
        target=np.matrix(self.from_transform(transform))
        func = lambda t: self.error_func(t, joint_names,target)
        
        optimization = fmin(func,joint_angles)  #fmin doesnt take dicts so we neet to split joint_angles up
        print(optimization)
        final_joint_angles ={joint_names[i]:optimization[i]for i in range(0,len(joint_names))}
        return final_joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        joint_angles = self.inverse_kinematics(effector_name,transform)
        names = self.chains[effector_name]
        times = [[0.,3.]]*len(names)
        # keys [angle now, [3,0,0] means going in straight line and beziere³ ,... angle we want
        keys= [[[self.perception.joint[name], [3, 0, 0], [3, 0, 0]], [joint_angles[name], [3, 0, 0], [3, 0, 0]]] for name in names]
        self.keyframes = (names, times, keys)  # the result joint angles have to fill in
   
    def from_transform(self, t):
        # return: [X,Y,Z,theta_X,theta_Y,theta_Z]
        theta_X = atan2(t[2,1],t[2,2])
        theta_Y = atan2(t[0,2],t[0,0])
        theta_Z = atan2(t[1,0],t[1,1])
        return [t[0,3],t[1,3],t[2,3],theta_X,theta_Y,theta_Z]
    
if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[1, 3] = 50
    T[2, 3] = -260
    agent.set_transforms('LLeg', T)
    
    agent.run()
