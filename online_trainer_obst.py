# Online trainerfor obstacle detection 

import time
import math


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

    def train(self, target):
        #position = self.robot.get_position()
        dist9,dist13,dist16,dist8,dist5,dist1=self.robot.get_obstacle() # On determine ici si le robot a rencontré un obstacle, les dist valent True soit 1 si il y a un objet 0 sinon
        #print(dist9,dist13,dist16,dist8,dist5,dist1)
        #Il faut mettre les inputs
        network_input = [0]*6 # 6 
        network_input[0] = dist9
        network_input[1] = dist13
        network_input[2] = dist16
        network_input[3]=dist8
        network_input[4]=dist5
        network_input[4]=dist1
        #Teta_t = 0
        print('run',self.running)
        
        while self.running:
            debut = time.time()
            command = self.network.runNN(network_input) # propage erreur et calcul vitesses roues instant t, voir si il y a bien deux arguments
            print('obstacle',self.robot.get_obstacle())
            alpha=1
            alpha_x = 1/6 
            alpha_y = 1/6
            alpha_teta = 1.0/(math.pi)
        
            # crit_av= alpha_x*alpha_x*(position[0])*(position[0]-target[0]) + alpha_y*alpha_y*(position[1]-target[1])*(position[1]-target[1]) + alpha_teta*alpha_teta*(position[2]-target[2]-theta_s(position[0], position[1]))*(position[2]-target[2]-theta_s(position[0], position[1]))  #avant commande
            crit_av= alpha*dist9**2+alpha*dist13**2+alpha*dist16**2+alpha*dist8**2+alpha*dist5**2+alpha*dist1**2 #avant commande, j'ai fait au pif à voir à quoi ça sert
                       
            self.robot.set_motor_velocity(command) # applique vitesses roues instant t                    
            time.sleep(0.050) # attend delta t
             
            dist9,dist13,dist16,dist8,dist5,dist1=self.robot.get_obstacle() # on obtient le nouvel état des capteurs à t+1
            # On modifie le réseau 
            # network_input[0] = (position[0]-target[0])*self.alpha[0]
            # network_input[1] = (position[1]-target[1])*self.alpha[1]
            # network_input[2] = (position[2]-target[2]-theta_s(position[0], position[1]))*self.alpha[2]
            
            network_input[0] = dist9
            network_input[1] = dist13
            network_input[2] = dist16
            network_input[3]=dist8
            network_input[4]=dist5
            network_input[4]=dist1 
            
            crit_ap= alpha*dist9**2+alpha*dist13**2+alpha*dist16**2+alpha*dist8**2+alpha*dist5**2+alpha*dist1**2 #apr�s commande, pareil j'aifait un peu au pf j'ai repris le ce qui avait été fait. 

            if self.training:#on est dans période d'entrainement
                delta_t = (time.time()-debut)

                grad = [
                    (-2/delta_t)*(alpha_x*alpha_x*(position[0]-target[0])*delta_t*self.robot.r*math.cos(position[2])
                    +alpha_y*alpha_y*(position[1]-target[1])*delta_t*self.robot.r*math.sin(position[2])
                    -alpha_teta*alpha_teta*(position[2]-target[2]-theta_s(position[0], position[1]))*delta_t*self.robot.r/(2*self.robot.R)),

                    (-2/delta_t)*(alpha_x*alpha_x*(position[0]-target[0])*delta_t*self.robot.r*math.cos(position[2])
                    +alpha_y*alpha_y*(position[1]-target[1])*delta_t*self.robot.r*math.sin(position[2])
                    +alpha_teta*alpha_teta*(position[2]-target[2]-theta_s(position[0], position[1]))*delta_t*self.robot.r/(2*self.robot.R))
                    ]

                # The two args after grad are the gradient learning steps for t+1 and t
                # si critere augmente on BP un bruit fction randon_update, sion on BP le gradient
                
                if (crit_ap <= crit_av) :
                    self.network.backPropagate(grad, 0.2,0.02) # grad, pas d'app, moment
                else :
                    #self.network.random_update(0.001)
                    self.network.backPropagate(grad, 0.2,0.02)
                
        self.robot.set_motor_velocity([0,0]) # stop  apres arret  du prog d'app
        #position = self.robot.get_position() #  obtient nvlle pos robot instant t+1
                #Teta_t=position[2]
             
                
        
        self.running = False
