import time
import math


def theta_s(x,y):
    if x>0:
        return math.atan(10*y)
    if x<=0:
        return math.atan(-10*y)

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

        self.alpha = [1/6]*6]

    def train(self, target):
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
            command = self.network.runNN(network_input)
            
            time.sleep(0.050)
            dist9,dist13,dist16,dist8,dist5,dist1=self.robot.get_obstacle()
            network_input[0] = dist9
            network_input[1] = dist13
            network_input[2] = dist16
            network_input[3]=dist8
            network_input[4]=dist5
            network_input[4]=dist1

            if self.training:
                delta_t = (time.time()-debut)

                grad = [
                    ((-1)/(delta_t**2))*(network_input[0]*delta_t*self.robot.r*math.cos(position[2])
                    +network_input[1]*delta_t*self.robot.r*math.sin(position[2])
                    -network_input[2]*delta_t*self.robot.r/(2*self.robot.R)),

                    ((-1)/(delta_t**2))*(network_input[0]*delta_t*self.robot.r*math.cos(position[2])
                    +network_input[1]*delta_t*self.robot.r*math.sin(position[2])
                    +network_input[2]*delta_t*self.robot.r/(2*self.robot.R))
                ]

                # The two args after grad are the gradient learning steps for t
                # and t-1
                self.network.backPropagate(grad, 0.05, 0)

        self.robot.set_motor_velocity([0,0])
self.running = False