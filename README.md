# Detec-obstacle
Deep learning et detection d'obstacle
Le projet est en cours le code n'est pas terminé !  
Ce code s'est inspiré de : https://github.com/sfosset/mines_olp

Pour lancer vrep depuis le terminal : 
Pour linux ./vrep.sh  -q simu_obstacle.ttt <br/>
Pour mac : ./vrep.app/Contents/MacOS/vrep  -q simu_obstacle.ttt 
Ensuite pour le connecter avec python : un le fichier run.py


## Lecture des capteurs : OK
Dans le fihier vrep_pioneer_simulation : j'ai défini la fonction get_obstacle qui utilise simxReadProximitySensor. Attention pour pouvoir interroger un capteur il faut avoir déclaré l'objet avant avec vrep.simxGetObjectHandle. 
La fonction pour le moment retourne la distance de trois capteurs 11,13 et 15 avec un objet (cf sur Vrep le robot). 
Cette fonction est ensuite utilisée dans le fichier online-trainer dans la méthode train associé à Online Trainer. L'idée est d'avoir à chaque coup de l'aprentissage l'état de ces capteurs. 
On a l'état des six capteurs : 1 si il y a en effet un objet détecté, 0 sinon.  
## Modification du NN 
Il faut je pense bien modifier le réseau de neurones avec 6 noeuds en entrée et deux noeuds en sortie.
J'ai pour l'instant pris le code **online trainer-new**  et je l'ai modifié pour essayer de l'adapter à notre problème. 
En entrée : Etats des 6 capteurs (0 ou 1). 1 correspond à la présence d'un obstacle
De ce que j'ai compris on prend l'état de nos capteurs (composé de 0 et de 1). Ensuite on calcule notre effet sur les roues du moteur grâce à notre réseau. On a ainsi un nouvel état des capteurs. Et on compare (fait la différence) de cet état avec l'état des capteurs lorsqu'aucun object est détecté (0,0,0,0,0,0). Cette différence correspond à l'erreur faite par notre réseau.  

Cela n'est pas fini, j'ai deux problèmes : 
- quelle est la forme de la fonction gradient ( il faut l'adapter à notre problème)
- quels sont les outputs du réseau (voir fichier **vrep_pioneer_simulation** dans l'init), comprendre à quoi il correspondent 
<br/>


## Test 
- Créer une scène obstacle : Fait il s'agit de **simu_obstacle.ttt** (disponible dans le dépôt)
- Test et analyse résultat
