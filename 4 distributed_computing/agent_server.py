'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
import time
import threading
from xmlrpc.server import SimpleXMLRPCServer
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', '3 kinematics'))

from inverse_kinematics import InverseKinematicsAgent


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self):

        super(ServerAgent, self).__init__()
        server = SimpleXMLRPCServer(("localhost",6969),allow_none=True)
        server.register_instance(self)
        server.register_function(self.get_angle,"get_angle")
        server.register_function(self.set_angle,"set_angle")
        server.register_function(self.get_posture,"get_posture")
        server.register_function(self.execute_keyframes,"execute_keyframes")
        server.register_function(self.get_transform,"get_transform")
        server.register_function(self.set_transform,"set_transform")
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.perception.joint[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        
        self.target_joints[joint_name] = angle
                
        # YOUR CODE HERE

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.recognize_posture(self.perception)

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        (names,times,keys)=keyframes
        duration = max(map(self.last,times)) # the end of the last motor movement
        self.keyframes = keyframes
        time.sleep(duration)            # block till last movement is executed

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.transforms[name]
    
    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.transforms[effector_name]=transform
    def last(self,array):
        return array[-1]
if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

