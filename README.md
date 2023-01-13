# Tennis Ball Collector

Ceci est un template de dépôt Git pour le cours d'ingénierie système et modélisation robotique à l'ENSTA Bretagne en 2023.


## Lancer la simulation

### Dépendences

###### A compléter avec la/les dépendences.
Utiliser ROS2 foxy

Installer gazebo pour foxy
```bash
sudo apt install ros-foxy-gazebo-ros-pkgs
```



### Démarrer la simulation

###### A compléter avec la/les commande(s) à lancer.

Pensez à sourcer votre ROS2 

```bash
colcon build --packages-select gillou tennis_court
```
```bash
source install/setup.bash
```
```bash
ros2 launch gillou display.launch.py
```


## Groupe

### Membres

Gace Hugo, Langlard Thibault, Potin Laurent, Reubrecht Hugo, Wanctin Hugo


### Gestion de projet

Lien vers le Taiga : https://tree.taiga.io/project/laurent_p-equipe-tennis/timeline



## Structure du dépôt

Ce dépôt doit être cloné dans le dossier `src` d'un workspace ROS 2.

### Package `tennis_court`

Le dossier `tennis_court` est un package ROS contenant le monde dans lequel le robot ramasseur de balle devra évoluer ainsi qu'un script permettant de faire apparaître des balles dans la simulation.
Ce package ne doit pas être modifié.
Consulter le [README](tennis_court/README.md) du package pour plus d'informations.

### Package 'gillou'

Le dossier 'gillou' est un package ROS contenant le robot. Lors du lancement de ce package celui-ci fait appelle au package 'tennis_court' pour générer le monde et place le robot sur le terrain.

### Documents

Le dossier `docs` contient tous les documents utiles au projet:
- Des [instructions pour utiliser Git](docs/GitWorkflow_fork.md)
- Un [Mémo pour ROS 2 et Gazebo](docs/Memo_ROS2.pdf)
- Les [slides de la présentation Git](docs/GitPresentation.pdf)


### Rapports

Le dossier `reports` doit être rempli avec les rapports d'[objectifs](../reports/GoalsTemplate.md) et de [rétrospectives](../reports/DebriefTemplate.md) en suivant les deux templates mis à disposition. Ces deux rapports doivent être rédigés respectivement au début et à la fin de chaque sprint.

## Caractéristiques du robot

-- Nom : Best Ball Catcher  

--taille (en mm): 
	corps 420x310x180 plaques épaisseur 6  
	roues r 100 w 40  
	pelle L 200  

-- masse totale entre 10kg et 15kg  
	corps 1.9kg  
	roues 200g par roue  
	pelle 1.5kg  

--matériaux :  
	roues pneus dentelés  
	corps bambou  
	pelle bambou  
