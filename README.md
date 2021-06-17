# Proces paletyzacji
## Skład grupy
* Klaudia Sagat
* Michał Heit
* Patryk Gawron

## Opis projektu

Projekt przedstawia proces paletyzacji, który składa się z trzech automatów: 
* automatu nadrzędnego związanego ze stanem palety oraz włączeniem procesu,
* automatu podrzędnego odpowiadającemu procesowi przenoszenia produktów na paletę,
* automatu podrzędnego ilustrującego stan lini taśmowej, po której przemieszczane są obiekty do spaletyzowania.

Poszczególne stany oraz przejścia między nimi w każdym z automatów przedstawiają poniższe diagramy. 

# Schemat procesu
![Graph](https://github.com/patrykGawron/TSWR_projekt/blob/master/diagrams.png)

## Build:
```bash
mkdir -p catkin_ws

cd ~/catkin_ws

sudo apt-get install ros-noetic-moveit-commander ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-moveit-ros-planning-interface ros-noetic-moveit-planners-ompl ros-noetic-joint-trajectory-controller ros-noetic-tf-conversions ros-noetic-ur-client-library ros-noetic-industrial-robot-status-interface ros-noetic-position-controllers ros-noetic-robot-state-publisher ros-noetic-tf2-tools ros-noetic-moveit-simple-controller-manager

git clone https://github.com/patrykGawron/TSWR_projekt.git

catkin_make

source devel/setup.bash

chmod +x ~/catkin_ws/src/my_robot_world/scripts/*
```

## Visualisation Launch:
```bash
roslaunch my_robot_world myRobot.launch
```
