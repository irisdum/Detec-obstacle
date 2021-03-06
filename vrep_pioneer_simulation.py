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
        self.scene = './simu_fleurs.ttt'
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
            #Les capteurs d'obstacles 1 2 3 4 5 6 7 8 (capteurs de l'avant du Pioneer)
            res,self.sensor_1=vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor1', vrep.simx_opmode_oneshot_wait) #il s'agit du capteur , vérifier le op_mode vrep.simx_opmode_blocking
            res,self.sensor_2=vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor2', vrep.simx_opmode_oneshot_wait) #il s'agit du capteur
            res,self.sensor_3=vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor3', vrep.simx_opmode_oneshot_wait) #il s'agit du capteur
            res,self.sensor_4=vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor4', vrep.simx_opmode_oneshot_wait) #il s'agit du capteur
            res,self.sensor_5=vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor5', vrep.simx_opmode_oneshot_wait) #il s'agit du capteur
            res,self.sensor_6=vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor6', vrep.simx_opmode_oneshot_wait) #il s'agit du capteur
            res,self.sensor_7=vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor7', vrep.simx_opmode_oneshot_wait) #il s'agit du capteur
            res,self.sensor_8=vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor8', vrep.simx_opmode_oneshot_wait) #il s'agit du capteur
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
        """ Returns a vector indicating if there are a obstacle detected, we have to consider 8 sensors """
      
        err_code1,detectionState1,detectedPoint1,detectedObjectHandle1,detectedSurfaceNormalVector1=vrep.simxReadProximitySensor(self.client_id,self.sensor_1,vrep.simx_opmode_streaming)
        err_code2,detectionState2,detectedPoint2,detectedObjectHandle2,detectedSurfaceNormalVector2=vrep.simxReadProximitySensor(self.client_id,self.sensor_2,vrep.simx_opmode_streaming)
        err_code3,detectionState3,detectedPoint3,detectedObjectHandle3,detectedSurfaceNormalVector3=vrep.simxReadProximitySensor(self.client_id,self.sensor_3,vrep.simx_opmode_streaming)
        err_code4,detectionState4,detectedPoint4,detectedObjectHandle4,detectedSurfaceNormalVector4=vrep.simxReadProximitySensor(self.client_id,self.sensor_4,vrep.simx_opmode_streaming)
        err_code5,detectionState5,detectedPoint5,detectedObjectHandle5,detectedSurfaceNormalVector5=vrep.simxReadProximitySensor(self.client_id,self.sensor_5,vrep.simx_opmode_streaming)
        err_code6,detectionState6,detectedPoint6,detectedObjectHandle6,detectedSurfaceNormalVector6=vrep.simxReadProximitySensor(self.client_id,self.sensor_6,vrep.simx_opmode_streaming)
        err_code7,detectionState7,detectedPoint7,detectedObjectHandle7,detectedSurfaceNormalVector7=vrep.simxReadProximitySensor(self.client_id,self.sensor_7,vrep.simx_opmode_streaming)
        err_code8,detectionState8,detectedPoint8,detectedObjectHandle8,detectedSurfaceNormalVector8=vrep.simxReadProximitySensor(self.client_id,self.sensor_8,vrep.simx_opmode_streaming)
        #print('Etat du capteur',err_code,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector)
        #print(detectionState,detectionState8,detectionState5,detectionState1,detectionState13,detectionState16)
        dist1=np.linalg.norm(detectedPoint1)
        dist2=np.linalg.norm(detectedPoint2)
        dist3=np.linalg.norm(detectedPoint3)
        dist4=np.linalg.norm(detectedPoint4)
        dist5=np.linalg.norm(detectedPoint5)
        dist6=np.linalg.norm(detectedPoint6)
        dist7=np.linalg.norm(detectedPoint7)
        dist8=np.linalg.norm(detectedPoint8)

        list_ind=[1,2,3,4,5,6,7,8]
        list_dist=[dist1,dist2,dist3,dist4,dist5,dist6,dist7,dist8]
        list_state=[detectionState1,detectionState2,detectionState3,detectionState4,detectionState5,detectionState6,detectionState7,detectionState8]
        return list_ind,list_dist,list_state

    


       