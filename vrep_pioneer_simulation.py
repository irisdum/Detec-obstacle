# coding: utf-8

import vrep
import math
import vrepConst
import numpy as np
def to_rad(deg):
    return 2*math.pi*deg/360

def to_deg(rad):
    return rad*360/(2*math.pi)

class VrepPioneerSimulation:
    def __init__(self):

        self.ip = '127.0.0.1'
        self.port = 19997
        self.scene = './simu_obstacle.ttt'
        self.gain = 2
        self.initial_position = [3,3,to_rad(45)]

        self.r = 0.096 # wheel radius
        self.R = 0.267 # demi-distance entre les r

        print('New pioneer simulation started')
        vrep.simxFinish(-1)
        self.client_id = vrep.simxStart(self.ip, self.port, True, True, 5000, 5)

        if self.client_id!=-1:
            print ('Connected to remote API server on %s:%s' % (self.ip, self.port))
            res = vrep.simxLoadScene(self.client_id, self.scene, 1, vrep.simx_opmode_oneshot_wait)
            res, self.pioneer = vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx', vrep.simx_opmode_oneshot_wait)
            res, self.left_motor = vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
            res, self.right_motor = vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)
            res,self.visible=vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_visible', vrep.simx_opmode_oneshot_wait)
            #Les capteurs d'obstacle 16,13, 9 et 8,5, 1
            res,self.sensor_16=vrep.simxGetObjectHandle(self.visible, 'Pioneer_p3dx_ultrasonicSensor16', vrep.simx_opmode_oneshot_wait) #il s'agit du capteur , vérifier le op_mode vrep.simx_opmode_blocking
            res,self.sensor_13=vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor13', vrep.simx_opmode_oneshot_wait) #il s'agit du capteur central
            res,self.sensor_9=vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor9', vrep.simx_opmode_oneshot_wait) #il s'agit du capteur
            res,self.sensor_8=vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor8', vrep.simx_opmode_oneshot_wait) #il s'agit du capteur
            res,self.sensor_5=vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor5', vrep.simx_opmode_oneshot_wait) #il s'agit du capteur
            res,self.sensor_1=vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor1', vrep.simx_opmode_oneshot_wait) #il s'agit du capteur
            self.set_position(self.initial_position)
            vrep.simxStartSimulation(self.client_id, vrep.simx_opmode_oneshot_wait)
           # print('sensor 1',self.sensor1)

        else:
            print('Unable to connect to %s:%s' % (self.ip, self.port))

    def set_position(self, position):
        """Set the position (x,y,theta) of the robot

        Args:
            position (list): the position [x,y,theta]
        """

        vrep.simxSetObjectPosition(self.client_id, self.pioneer, -1, [position[0], position[1], 0.13879], vrep.simx_opmode_oneshot_wait)
        vrep.simxSetObjectOrientation(self.client_id, self.pioneer, -1, [0, 0, to_deg(position[2])], vrep.simx_opmode_oneshot_wait)

    def get_position(self):
        """Get the position (x,y,theta) of the robot

        Return:
            position (list): the position [x,y,theta]
        """
        position = []
        res, tmp = vrep.simxGetObjectPosition(self.client_id, self.pioneer, -1, vrep.simx_opmode_oneshot_wait)
        position.append(tmp[0])
        position.append(tmp[1])

        res, tmp = vrep.simxGetObjectOrientation(self.client_id, self.pioneer, -1, vrep.simx_opmode_oneshot_wait)
        position.append(tmp[2]) # en radian

        return position

    def set_motor_velocity(self, control):
        """Set a target velocity on the pioneer motors, multiplied by the gain
        defined in self.gain

        Args:
            control(list): the control [left_motor, right_motor]
        """
        vrep.simxSetJointTargetVelocity(self.client_id, self.left_motor, self.gain*control[0], vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetVelocity(self.client_id, self.right_motor, self.gain*control[1], vrep.simx_opmode_oneshot_wait)

    def get_obstacle(self):
        """ Returns a vector indicating if there are a obstacle detected, we have to consider 6 sensors : 3 from a side 3 from the other"""
        #print('je suis dans obstacle')
        # sensor=[0]*16 #the defaut value : nothing detected
        # for i in range(len(2)) : #we have 16 sensors detectors
        #    print(vrep.simHandleProximitySensor(sim_handle_all_except_explicit))
        # print('handle sensor',vrep.simHandleProximitySensor(self.visible))
        err_code,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(self.client_id,self.sensor_9,vrep.simx_opmode_streaming)
        err_code8,detectionState8,detectedPoint8,detectedObjectHandle8,detectedSurfaceNormalVector8=vrep.simxReadProximitySensor(self.client_id,self.sensor_8,vrep.simx_opmode_streaming)
        err_code5,detectionState5,detectedPoint5,detectedObjectHandle5,detectedSurfaceNormalVector5=vrep.simxReadProximitySensor(self.client_id,self.sensor_5,vrep.simx_opmode_streaming)
        err_code1,detectionState1,detectedPoint1,detectedObjectHandle1,detectedSurfaceNormalVector1=vrep.simxReadProximitySensor(self.client_id,self.sensor_1,vrep.simx_opmode_streaming)
        err_code13,detectionState13,detectedPoint13,detectedObjectHandle13,detectedSurfaceNormalVector13=vrep.simxReadProximitySensor(self.client_id,self.sensor_13,vrep.simx_opmode_streaming)
        err_code16,detectionState16,detectedPoint16,detectedObjectHandle16,detectedSurfaceNormalVector16=vrep.simxReadProximitySensor(self.client_id,self.sensor_16,vrep.simx_opmode_streaming)
        #print('Etat du capteur',err_code,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector)
        #print(detectionState,detectionState8,detectionState5,detectionState1,detectionState13,detectionState16)
        dist9=is_obstacle(np.linalg.norm(detectedPoint))
        dist13=is_obstacle(np.linalg.norm(detectedPoint13))
        dist16=is_obstacle(np.linalg.norm(detectedPoint16))
        dist8=is_obstacle(np.linalg.norm(detectedPoint8))
        dist5=is_obstacle(np.linalg.norm(detectedPoint5))
        dist1=is_obstacle(np.linalg.norm(detectedPoint1))
        return detectionState,detectionState8,detectionState5,detectionState1,detectionState13,detectionState16
        #print('dist', dist11)

def is_obstacle(dist):
    """ Knowing the distance between the orbot and the next obstacle consider if there is an obstacle or not"""
    if dist>0.5 :
        return 0
    else :
        return 1
        # print('Etat capteur',vrep.simxReadProximitySensor(self.client_id,self.sensor_cent,vrepConst.simx_opmode_streaming ))
        # print('Etat capteur',vrep.simxReadProximitySensor(self.client_id,self.sensor_1,vrepConst.simx_opmode_streaming ))
        # print('Etat capteur',vrep.simxReadProximitySensor(self.client_id,self.sensor_2,vrepConst.simx_opmode_streaming ))
