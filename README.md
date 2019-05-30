# Detec-obstacle
Deep learning et detection d'obstacle
Le projet est en cours le code n'est pas terminé !  
Ce code s'est inspiré de : https://github.com/sfosset/mines_olp

Pour lancer vrep depuis le terminal : 
Pour linux ./vrep.sh  -q myScene.ttt
Pour mac : ./vrep.app/Contents/MacOS/vrep  -q ../../../myScene.ttt
Ensuite pour le connecter avec python : un le fichier run.py

## Lecture des capteurs : 
utilisation de la fonction simxReadProximitySensor définit dans le fichier vrep.py afin de déterminer l'état d'un capteur
Pour cela j'ai crée une méthode get_obstacle dans le fichier vrep-pionner-simulation. Cette fonction a pour but de retourner l'état des trois capteurs étudiés. 
Cette fonction est ensuite utilisé dans le fichier online-trainer dans la méthode train associé à Online Trainer. L'idée est d'avoir à chaque coup de l'aprentissage l'état de ces capteurs. 

Pour le moment ce qui pose problème c'est la lecture simxReadProximitySensor. Je pense que je n'e l'utilise pas encore bien, je n'arrive pas à obtenir les bonnes informations encore. 
