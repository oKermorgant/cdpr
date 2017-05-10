#!/usr/bin/python

'''
This node subscribes to a 32-length array and plots the corresponding polygon and barycenter
'''




class Listener:
    def __init__(self):
        # matrices
        self.msg_ok = False
        self.H = pl.zeros((8,2))
        self.A = pl.zeros((8,1))
        self.B = pl.zeros((8,1))
        
        # subscriber to H-A-B values
        rospy.Subscriber('barycenter', Float32MultiArray, self.read_hab)
        
    def read_hab(self, msg):
        1
        
        
        
        
        
        
        
if __name__ == '__main__':
    '''
    Begin of main code
    '''    
    
    rospy.init_node('barycenter')


    
