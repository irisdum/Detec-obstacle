# Online trainerfor obstacle detection

import time
import math
import numpy as np
from Brait_vit import*


def theta_s(x,y):
    if x>0:
        return 1*math.atan(1*y)
    if x<=0:
        return 1*math.atan(-1*y)

class OnlineTrainer:
    def __init__(self, robot, NN):
        """
        Args:
            robot (Robot): a robot instance following the pattern of
                VrepPioneerSimulation
            target (list): the target position [x,y,theta]
        """
        self.robot = robot
        self.network = NN

        self.alpha = [1/6,1/6,1/(math.pi)]  # normalition avec limite du monde cartesien = -3m � + 3m

    def train(self):
        #position = self.robot.get_position()
        list_ind,list_dist,list_state=self.robot.get_obstacle() # On determine ici si le robot a rencontré un obstacle, les dist valent True soit 1 si il y a un objet 0 sinon
        #print(dist9,dist13,dist16,dist8,dist5,dist1)
        #Il faut mettre les inputs
        network_input = [0]*6 # 6
        network_input=list_dist
        #Teta_t = 0
        #print('run',self.running)
        self.robot.set_motor_velocity([2,2])
        while self.running:
            debut = time.time()
            command = self.network.runNN(network_input) # propage erreur et calcul vitesses roues instant t, voir si il y a bien deux arguments
            #print('obstacle',self.robot.get_obstacle())
            #print(command)
            alpha=1/6
            alpha_x = 1/6
            alpha_y = 1/6
            alpha_teta = 1.0/(math.pi)

            # crit_av= alpha_x*alpha_x*(position[0])*(position[0]-target[0]) + alpha_y*alpha_y*(position[1]-target[1])*(position[1]-target[1]) + alpha_teta*alpha_teta*(position[2]-target[2]-theta_s(position[0], position[1]))*(position[2]-target[2]-theta_s(position[0], position[1]))  #avant commande
            crit_av= np.sum([elem**2 for elem in network_input])#avant commande, j'ai fait au pif à voir à quoi ça sert

            self.robot.set_motor_velocity(command) # applique vitesses roues instant t
            time.sleep(0.050) # attend delta t

            list_ind2,list_dist2,list_state2=self.robot.get_obstacle() # on obtient le nouvel état des capteurs à t+1

            network_input=list_dist2
            crit_av= np.sum([elem**2 for elem in network_input])
           #apr�s commande, pareil j'aifait un peu au pf j'ai repris le ce qui avait été fait.

            if self.training:#on est dans période d'entrainement
                delta_t = (time.time()-debut)
                # On calcule la vitesse normalement voulue
                time.sleep(0.060)
                vleft,vright=calc_vit_brait(list_ind,list_dist,list_state)#les vitesses calculées par Braitenberg
                grad=[command[0]-vleft,command[1]-vright]
                #print(grad)
                self.network.backPropagate(grad, 0.05,0.2)
                time.sleep(0.050) # attend delta t
        self.robot.set_motor_velocity([0,0]) # stop  apres arret  du prog
        self.running = False
        #position = self.robot.get_position() #  obtient nvlle pos robot instant t+1
                #Teta_t=position[2]
