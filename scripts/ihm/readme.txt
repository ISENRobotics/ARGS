Lancement de l'IHM:
 - Démarrer le node de la caméra USB : roslaunch usb_cam-test.launch
 - Démarrer le serveur vidéo : rosrun web_video_server web_video_server
 - Demarer server_web.py : python server_web.py
 - Le serveur est lancé et l'IHM est accessible à l'adresse : http://localhost:8888/index.py

 Informations:
 - L'IHM est paramétrée pour streamer le flux vidéo du topic : /augmented_reality_output/image_raw qui est de type compressed
 - Le topic peut être changé dans le code du fichier index.py ligne 83
 - La configuration actuelle permet d'accéder au node web_video_server lancé sur le TurtleBot avec la configuration suviante: http://192.168.2.126:8182
 - Le port et l'adresse IP sont modifiables dans le fichier /web_video_server/src/web_video_server.cpp en modifiant le paramètre address ligne 50 et port ligne 47


Le template de l'IHM utilisé est :

Theme Name: Knight
Theme URL: https://bootstrapmade.com/knight-free-bootstrap-theme/
Author: BootstrapMade
Author URL: https://bootstrapmade.com
