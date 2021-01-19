# Mapas con pcl

## 1.

Se corre *roslaunch openni_launch openni.launch*, permite aceder a la información del Kinect. 

## 2.

Se corre *rosrun mapeo_pkg catch_point_cloud*. El archivo .py se encuentra en el directorio */scripts*.
Este nodo se subscribe al topic *camera/depth_registered/points* para obtener la nube de puntos rgb-d del kinect;
se lee la entrada del terminal para determinar la nube de puntos "Source" ('s') y "Target" ('t') con las que se realizará 
el algoritmo de ICP. 

## 3

Se corre *rosrun mapeo_pkg point_cloud_map*. El archvivo .cpp se encuentra en el directorio */src*.
Este nodo recibe las nubes de puntos "Source" y "Target" y realiza el procesamiento de ICP, la nube de puntos
que es acumulada en cada toma se publica en el topic *point_cloud_map*. 

## 4 

Se corre *rosrun rviz rviz* y se visualiza la nube de puntos del topic *point_cloud_map*
