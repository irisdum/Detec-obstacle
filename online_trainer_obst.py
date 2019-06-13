# Online trainerfor obstacle detection 

import time
import math
import numpy as np
from Brait_vit import*




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
        
        list_ind,list_dist,list_state=self.robot.get_obstacle() # On determine ici si le robot a rencontré un obstacle, les dist valent True soit 1 si il y a un objet 0 sinon
      
        #Il faut mettre les inputs
        network_input = [0]*6 
        network_input=list_dist
       
        
        while self.running:
            debut = time.time()
            command = self.network.runNN(network_input) # propage erreur et calcul vitesses roues instant t, voir si il y a bien deux arguments
          
            self.robot.set_motor_velocity(command) # applique vitesses roues instant t                    
            time.sleep(0.050) # attend delta t
             
            list_ind2,list_dist2,list_state2=self.robot.get_obstacle() # on obtient le nouvel état des capteurs à t+1
            
            network_input=list_dist2
      
           #apr�s commande, pareil j'aifait un peu au pf j'ai repris le ce qui avait été fait. 

            if self.training:#on est dans période d'entrainement
                delta_t = (time.time()-debut)
                # On calcule la vitesse normalement voulue 
                time.sleep(0.060)
                vleft,vright=calc_vit_brait(list_ind,list_dist,list_state)#les vitesses calculées par Braitenberg
                grad=[command[0]-vleft,command[1]-vright]
                #propagation de l'erreur
                self.network.backPropagate(grad, 0.05,0.2)
                time.sleep(0.050) # attend delta t
        self.robot.set_motor_velocity([0,0]) # stop  apres arret  du prog
        self.running = False
        
             

