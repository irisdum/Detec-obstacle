#run test for braitenberg avoid obstacle

import json
import threading
import sys, os
sys.path.append('~/Desktop/Projets en cours/Robotique_projet/Detec-obstacle')#/Users/iris/Desktop/APP-EL_Pioneer-VREP-fonctionneTB')
from rdn import Pioneer # rdn pour ROS avec le pioneer
from vrep_pioneer_simulation import VrepPioneerSimulation
from test_brait import*
robot = VrepPioneerSimulation()



trainer = Obstacle_test(robot)
choice = ''
while choice!='y' and choice !='n':
    choice = input('Do you want to moove the robot ? (y/n) --> ')

if choice == 'y':
    trainer.training = True
elif choice == 'n':
    trainer.training = False

continue_running = True

while(continue_running):

    thread = threading.Thread(target=trainer.moove, args=(100,))
    trainer.running = True
    thread.start()

    #Ask for stop running
    input("Press Enter to stop the current training")
    trainer.running = False
    choice = ''
    while choice!='y' and choice !='n':
        choice = input("Do you want to continue ? (y/n) --> ")

    if choice == 'y':
        choice_learning = ''
        while choice_learning != 'y' and choice_learning !='n':
            choice_learning = input('Do you want to learn ? (y/n) --> ')
        if choice_learning =='y':
            trainer.training = True
        elif choice_learning == 'n':
            trainer.training = False
        
    elif choice == 'n':
        continue_running = False
