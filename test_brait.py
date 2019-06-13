import math

class Obstacle_test:
    def __init__(self, robot):
        """
        Args:
            robot (Robot): a robot instance following the pattern of
                VrepPioneerSimulation
            target (list): the target position [x,y,theta]
        """
        self.robot = robot

        self.alpha = [1/6,1/6,1/(math.pi)]  # normalition avec limite du monde cartesien = -3m � + 3m
        
    def avoid_obst(self):
        noDetectionDist=0.5
        maxDetectionDist=0.2
        detect=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] #
        braitenbergL=[-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0] 
       braitenbergR=[-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        v0=1 # what is v0 and why is it set to 2, does it mean a velocity of 2m/s or something

        list_ind,list_dist,list_state=self.robot.get_obstacle()  
        
        for i in range(len(list_ind)):
            dist=list_dist[list_ind[i]-1]
            if(list_state[i] and dist<noDetectionDist):
                print('un objet', str(list_ind[i]))
                if (dist<maxDetectionDist) :
                    dist=maxDetectionDist
                detect[list_ind[i]]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
    #   simHandleChildScript(sim_handle_all_except_explicit)
    
        vLeft=v0
        vRight=v0
    
        for i in range(0,16):
            vLeft=vLeft+braitenbergL[i]*detect[i]
            vRight=vRight+braitenbergR[i]*detect[i]
    
        self.robot.set_motor_velocity([vLeft,vRight])
        self.running = False
        
    def moove(self,iter):
        for i in range(iter):
            self.avoid_obst()