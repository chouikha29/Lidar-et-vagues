# Estimation de la Hauteur et de la Direction des Vagues par Analyse d'Image LIDAR

### Objectif du Projet
L’objectif principal de ce projet est de déterminer la faisabilité et l’efficacité d’utiliser les images LIDAR pour obtenir des informations sur l’état de la mer, en se concentrant spécifiquement sur l’estimation de la direction et de la hauteur des vagues.


### Options disponibles :
- `-h`, `--help` : Affiche ce message d'aide et quitte.
- `--lidar_vel LIDAR_FILE_PATH NUM_FRAME_TO_EXTRACT` : Lecture d'un fichier .pcap de Lidar Velodyne.
- `--lidar_ous LIDAR_FILE_PATH JSON_META_FILE_PATH NUM_FRAME_TO_EXTRACT` : Lecture d'un fichier .pcap de Lidar Ouster.
- `--simu SEA_TYPE NBR_FRAMES` : Génère une mer simulée.
- `--gyro CSV_FILE_PATH` : Lecture d'un fichier CSV de l'IMU.
- `--corr YPR_OPTION` : Correction du nuage de points avec les données de l'IMU (Yaw, Pitch, Roll).
- `--prefilter JSON_FILE_PATH` : Filtre le nuage de points avant correction.
- `--postfilter JSON_FILE_PATH` : Filtre le nuage de points après correction.
- `--display DISPLAY_TYPE` : Affiche les données avec les options suivantes :
  - `pc` : Nuage de points
  - `mesh` : Génération de maillage
 

---

## Guide d'Installation

### Version de Python
Assurez-vous d'avoir **Python 3.10.4** ou une version plus récente installée.

### Étapes d'Installation

1. **Cloner le dépôt :**
   ```bash
   git clone https://github.com/chouikha29/Lidar-et-vagues.git
2. **Accéder au branche master:**
    ```bash
    git checkout master
3. **Installer les dépendances :**
    ```bash
    pip install .
4. **Avec Conda :**
   Si vous préférez utiliser Conda, créez un environnement Conda en utilisant le fichier de configuration :
    ```bash
    conda env create --file=Conda/env.yaml

### Vérification de l'Installation
    python ./LidarDataProc/LidarDataProc.py --help
**Si le message d'aide s'affiche (cela peut prendre quelques secondes pour importer les modules), votre installation est terminée avec succès. Félicitations !**



   
