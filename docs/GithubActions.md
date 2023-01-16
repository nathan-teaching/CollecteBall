# GitHub Actions

Pour mettre en place de l'Intégration Continue (CI) dans GitHub

## Set Up

1. Dans son dépôt local, créer le répertoire `.github/workflows/` pour stocker les fichiers de workflow.

2. Dans ce répertoire crée un fichier exemple :  `tennis-court-foxy.yaml`

   ```yaml
   name: tennis-court-foxy
   
   on: 
     push:
       branches: [ main ]
     pull_request:
       branches: [ main ]
       
   jobs:
     build:
       runs-on: ubuntu-20.04
       steps:
         - uses: ros-tooling/setup-ros@v0.3
         - uses: ros-tooling/action-ros-ci@0.2.7
           with:
             target-ros2-distro: foxy
             package-name: your_package_name
   ```



3. Valider les modifications et poussez vers sur le dépôt sur GitHub 



## A voir

Exemple d'actions  :
 ```yaml
   # Nom de la github actions
   name: test-pkg 
   
   # Quand executer les actions
   on:  
     push:
       branches: [ main, devel ]
     pull_request:
       branches: [ main ]
   
   # définir les actions à faire :
   # - Mettre en place ce qu'il faut pour tester
   # - tester 
   jobs:   
     build:
     
       runs-on: ubuntu-latest # initialisation de l'os
       container: # si on veut executer dans un container (necessite runs-on qui hebergera alors le container)
       	image: # docker image
       
       steps:
         - uses: pkg/routine@version # pour utiliser une github action de la marketplace
         - run: # command bash
         - name: # si on veut donner un nom spécifique à cette commande
           uses: # utiliser 
           with:
             parametres: # si besoin d'avoir des paramètres
         - run : # run test
 ```
Pour définir plusieurs environnements il y a la possibilité de passer des paramètres via un système matriciel. Ici on lancerai des tests sur *ubuntu-latest avec foxy* et sur *ubuntu-latest avec humble* par exemple. Voir la [doc](https://docs.github.com/fr/actions/using-jobs/using-a-matrix-for-your-jobs)

```yaml
strategy:
  matrix:
  	os: [ubuntu-latest]
  	versions: [foxy, humble]
```

Voir les différentes routines qui existent dans le [Github Action Marketplace](https://github.com/marketplace?type=actions)

Pour Checkout  à un certain commit : https://github.com/actions/checkout

### Sources

[Doc GitHub](https://docs.github.com/fr/actions/learn-github-actions/understanding-github-actions)

[ROS 2 CI Action](https://github.com/marketplace/actions/ros-2-ci-action#Requirements)

[ROS 2 CI  Set-up](https://github.com/marketplace/actions/setup-ros-environment)
