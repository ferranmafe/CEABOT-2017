# CEABOT-2017
Code developed at IRI (Institut de Robòtica i Informàtica Industrial) in order to participate in CEABOT competition. [CEABOT](www.ceabot.es) is a humanoid robot competition at Spanish level, in which undergraduate
students compete with each other in order to pass different kind of tasks, explained below.

To participate in the competition, IRI used 2 robots: [Darwin](https://ro-botica.com/es/Producto/ROBOTIS-DARwIn-OP-Deluxe/) (expensive, software based on ROS) and [Bioloid](https://www.ro-botica.com/Producto/ROBOTIS-PREMIUM-Kit-educativo-Bioloid/) (cheap, software based on C). Darwin was used in was harder to break the robot.

## Authors
* Marc Domínguez de la Rocha
* Ferran Martínez Felipe

## Dependencies
* gcc
* ROS Indigo
* IRI software. This code is currently private.

## Installation
### Darwin tasks (maze, stairs, vision)
This code is based on ROS. In order to use the software, IRI software is needed, so the installation process
is not going to be explained.

### Bioloid tasks (sumo)
As in the previous case, this software relies on IRI software, so the installation process
is not going to be explained.

## Task Description
### Darwin
#### Maze
Task designed to go out to a labyrinth relying on computer vision. The maze had the walls tagged with QR codes describing the position of the wall, so developing an heuristics to go outside the labyrinth was developed.

#### Vision
Task in which the robot had to follow different QR codes placed around the robot, in which each QR code described the position of the next one.

#### Stairs
Task in which the robot had to go up and down a set of stairs. To develop this activity, a set of sensors were placed on robot feet

### Bioloid
#### Sumo
Task in which the robot had to push other robot to the ground in order to win. This task was a tournament.
