# Parallel-Link Arm Robot

Arduino-based parallel-link robotic arm with a complete control system 
and a mathematical framework for its kinematics.

## Overview

This project includes a kinematic model of the manipulator (3D geometric/
kinematic model), direct and inverse kinematics derivations, and an 
Arduino-based control system with three operating modes.

## Demo

<img width="380" height="214" alt="output" src="https://github.com/user-attachments/assets/019f75e8-0c22-4d2c-b952-6abfd4069091" />

[Watch the full video](demo.mp4)

## Features

- Manual control of the manipulator
- Automatic object search and grasping using an ultrasonic sensor
- Precise coordinate input mode for moving the end effector 
  (inverse kinematics)

## Repository Structure

- `/src` — control system code (C++, Arduino)
- `/docs` — mathematical derivations, kinematics calculations, and 
  model description

## Technologies

Arduino (C++), GeoGebra (3D kinematic model), parallel kinematics 
mathematical modeling

## Model

<img width="957" height="403" alt="image" src="https://github.com/user-attachments/assets/dff865a9-81bc-40d4-954a-70ef1712d2fa" />

<img width="1002" height="396" alt="image" src="https://github.com/user-attachments/assets/f568cd5e-9bb4-4cc8-b363-0f3cc2240b5d" />

## Circuit diagram

<img width="652" height="662" alt="Schematics" src="https://github.com/user-attachments/assets/4b6a3252-5276-47f0-8aa4-8e2aef78a124" />

## Project Status

Educational/research project focused on kinematic modeling and 
control algorithms.
