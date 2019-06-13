import math
import time
class Obstacle_test:
    def __init__(self, robot):
        """
        Args:
            robot (Robot): a robot instance following the pattern of
                VrepPioneerSimulation
        """
        self.robot = robot

        self.alpha = [1/6,1/6,1/(math.pi)]  # normalisation avec limite du monde cartesien = -3m + 3m

    def avoid_obst(self):

        noDetectionDist=0.35 #distance a partir de laquelle l'obstacle est pris en compte
        maxDetectionDist=0.15 #distance max a laquelle s'approcher

        detect=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        braitenbergL=[0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8, 0,0,0,0,0,0,0,0] #parametres de braitenberg pour la roue gauche
        braitenbergL=[-2*elem for elem in braitenbergL]
        braitenbergR=[0.8,0.7,0.6,0.5,0.4,0.3,0.2,0.1, 0,0,0,0,0,0,0,0] #parametres de braitenberg pour la roue droite
        braitenbergR=[-2*elem for elem in braitenbergR]

        v0=2

        list_ind,list_dist,list_state=self.robot.get_obstacle()

        for i in range(len(list_ind)):
            dist=list_dist[list_ind[i]-1]
            if(list_state[i] and dist<noDetectionDist):
                print('un objet', str(list_ind[i]))
                if (dist<maxDetectionDist) :
                    dist=maxDetectionDist
                detect[list_ind[i]]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist))


        vLeft=v0
        vRight=v0

        for i in range(0,16): #changement des vitesses
            vLeft=vLeft+braitenbergL[i]*detect[i]
            vRight=vRight+braitenbergR[i]*detect[i]

        self.robot.set_motor_velocity([vLeft,vRight])
        self.running = False

    def moove(self,iter):
        for i in range(iter):
            self.avoid_obst()
            time.sleep(0.050)
