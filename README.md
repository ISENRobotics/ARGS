# Autonomous Robots Gaming System

## Launch :
Environnement sur le PC:  
export ROS_MASTER_URI=http://IP_ROBOT:PORT_ROBOT  
export ROS_HOSTNAME=IP_PC  

Environnement sur le robot:  
export ROS_MASTER_URI=http://IP_ROBOT:PORT_ROBOT  
export ROS_HOSTNAME=IP_ROBOT  

Sur le robot:  
 - roslaunch turtlebot_bringup minimal.launch // Démarre roscore  
 - roslaunch turtlebot_bringup 3dsensor.launch // Démarre la kinect  
 - roslaunch usb_cam usb_cam-test.launch // Démarre le node de la caméra USB sur le TurtleBot qui stream la scène  
 - rosrun args script/augmented_reality.py // Pour le lancement du node de realitée augmentée  
 - rosrun web_video_server web_video_server // Démarre le serveur vidéo sur le TurtleBot qui stream la scène  
 - rosrun args server_web.py // Demarre le serveur web python pour lancer l'IHM  
 - Le serveur est lancé et l'IHM est accessible à l'adresse : http://localhost:8888/index.py  

## Informations:  
 - L'IHM est paramétrée pour streamer le flux vidéo du topic : /augmented_reality_output/image_raw qui est de type compressed  
 - Le topic peut être changé dans le code du fichier index.py ligne 83  
 - La configuration actuelle permet d'accéder au node web_video_server lancé sur le TurtleBot avec la configuration suviante: http://192.168.2.126:8182  
 - Le port et l'adresse IP sont modifiables dans le fichier /web_video_server/src/web_video_server.cpp en modifiant le paramètre address ligne 50 et port ligne 47  
 - Un fichier launch est créé mais non fonctionnel  

## Notes :
augmented_reality.py script needs scipy, for that :  
pip install scipy (if failed, may needs a pip upgrade)  




## Le template de l'IHM utilisé est :

Theme Name: Knight
Theme URL: https://bootstrapmade.com/knight-free-bootstrap-theme/
Author: BootstrapMade
Author URL: https://bootstrapmade.com
