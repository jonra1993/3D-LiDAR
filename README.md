## The nearest object localization through 3D lidar reconstruction using an embedded system

This work was presented in the XXVII Electrical and Electronics journey of Escuela Politécnica Nacional.

## Abstract

This work presents a system capable of reconstructing a three-dimensional environment and from its information it finds where the nearest object is located. The 3D information, which was acquired through a LiDAR (Light Detection and Ranging) sensor, was processed as a point cloud using the python binding to the Point Cloud Library. The algorithms used in this work include PassThrough filter, statistical outlier removal and RANSAC. Finally, a kd-tree algorithm was used to find the closest points to the system and in this way, it is possible to find the nearest object. The developed system has as main devices a Hokuyo laser sensor and a Raspberry Pi 3. It was tested in indoor environments. The results show that the system can effectively locate the nearest object.

You can download the  [Paper](http://ciecfie.epn.edu.ec/wss/VirtualDirectories/80/JIEE/historial/XXVII/Contenido/MEMORIAS_XXVII-31-37.pdf) .

<br>

## Components
<p align="center">
  <img height="320" width="480" src="https://www.jonathanvargas.ml/wp-content/uploads/2019/03/1Parts.png">
</p>

<br>

## Prototype
<p align="center">
  <img height="200" width="480" src="https://www.jonathanvargas.ml/wp-content/uploads/2019/03/2Prototype.png">
</p>

<br>

## 3D Scan
<p align="center">
  <img height="200" width="480" src="https://www.jonathanvargas.ml/wp-content/uploads/2019/03/5OriginalScene_Total.png">
</p>

<br>

## Result after filtering
<p align="center">
  <img height="300" width="480" src="https://www.jonathanvargas.ml/wp-content/uploads/2019/03/10Nearest.png">
</p>


## License

[![License](http://img.shields.io/:license-mit-blue.svg?style=flat-square)](http://badges.mit-license.org)

- **[MIT license](http://opensource.org/licenses/mit-license.php)**
- Copyright 2019 © <a href="https://www.jonathanvargas.ml" target="_blank">Jonathan Vargas</a>.

