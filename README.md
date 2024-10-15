# Projet ROS 2 : Système d'Éclairage Intelligent

Ce projet met en œuvre un système d'éclairage intelligent à l'aide de ROS 2. Il se compose de plusieurs composants, y compris un hub central, la simulation de capteurs, et une interface graphique pour contrôler la température et l'humidité.

## Structure du Projet

Le projet est divisé en plusieurs packages ROS 2 :

### 1. **central_ai_hub**
Le package **`central_ai_hub`** est le cœur du système. Il collecte les données des capteurs (température et humidité) et ajuste la couleur de l'éclairage et le contrôle de l'humidité en fonction de ces données.
- **Publications** :
  - `light_color` : Ajuste la couleur de l'éclairage (chaud ou froid) en fonction de la température.
  - `humidity_control` : Contrôle l'humidité en activant ou désactivant le déshumidificateur selon le taux d'humidité.
- **Abonnements** :
  - `temperature_data` : Reçoit les données de température.
  - `humidity_data` : Reçoit les données d'humidité.

### 2. **multi_sensor_array**
Le package **`multi_sensor_array`** simule les capteurs en publiant des données de température et d'humidité à intervalles réguliers.
- **Publications** :
  - `temperature_data` : Publie les données de température.
  - `humidity_data` : Publie les données d'humidité.

### 3. **room_control_nodes**
Le package **`room_control_nodes`** gère le contrôle d'une pièce spécifique, ajustant l'éclairage en fonction des capteurs. Chaque pièce a son propre nœud de contrôle.

### 4. **sensor_control_gui**
Le package **`sensor_control_gui`** fournit une interface graphique (développée avec **PyQt5**) permettant à l'utilisateur de modifier manuellement la température et l'humidité.
- **Contrôles** :
  - Slider pour ajuster la température.
  - Slider pour ajuster l'humidité.
- **Publications** :
  - `temperature_data` : Envoie les valeurs de température modifiées par l'utilisateur.
  - `humidity_data` : Envoie les valeurs d'humidité modifiées par l'utilisateur.

### 5. **lighting_system_launch**
Le package **`lighting_system_launch`** contient les fichiers de lancement ROS 2 qui permettent de démarrer tous les nœuds du système.

## Lancement du Système

Pour lancer le système complet, exécutez la commande suivante :

```bash
ros2 launch lighting_system_launch lighting_system_launch.py
```

