# Detec-obstacle
Deep learning et detection d'obstacle
Le projet est en cours le code n'est pas terminé !  
Ce code s'est inspiré de : https://github.com/sfosset/mines_olp

Pour lancer vrep depuis le terminal : 
Pour linux ./vrep.sh  -q myScene.ttt <br/>
Pour mac : ./vrep.app/Contents/MacOS/vrep  -q ../../../myScene.ttt
Ensuite pour le connecter avec python : un le fichier run.py

## Lecture des capteurs : OK
Dans le fihier vrep_pioneer_simulation : j'ai défini la fonction get_obstacle qui utilise simxReadProximitySensor. Attention pour pouvoir interroger un capteur il faut avoir déclaré l'objet avant avec vrep.simxGetObjectHandle. 
La fonction pour le moment retourne la distance de trois capteurs 11,13 et 15 avec un objet (cf sur Vrep le robot). 
Cette fonction est ensuite utilisée dans le fichier online-trainer dans la méthode train associé à Online Trainer. L'idée est d'avoir à chaque coup de l'aprentissage l'état de ces capteurs. 

Il reste cependant à refaire le même travail avec les 3 autres capteurs de l'autre côté 
## Modification du NN 
Il faut je pense bien modifier le réseau de neurones avec 6 noeuds en entrée et deux noeud en sortie
En entrée N capteurs en sortie : moteur droite et moteur gauche
Le but est d'éloigner le robot de l'obstacel
Définir la fonction coût (voir articles)

## Test 
Créer une scène obstacle
Test et analyse résultat
