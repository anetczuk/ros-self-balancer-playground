# Robot Operating System two wheel self-balancing robot playground

This is environment for self-balancing robot. It consists of simulation model and control algorithms.


## How to build:

1. configure catkin workspace by calling script: *initialize_workspace.sh*
2. start virtual environment by *startenv.sh*
3. build projects by *make.sh*


## How to run:

Start every command inside virtual environment, each in new command line:
1. start core: ```roscore```
2. launch TeeterBot simulation *src/teeterbot/run_empty.sh*
3. start *self-balancer* driver by *src/self-balancer/balancer-driver/src/drive*
4. (optional) start *self-balancer* GUI by ```rqt```
5. (optional) run one of plotters *src/self-balancer/balancer-driver/plot*.sh* to observer inputs and outputs


## Subprojects

Playground consists of following subprojects:
1. [self-balancer](src/self-balancer/README.md) -- varius control algorithms for the robot
2. [TeeterBot](src/teeterbot/README.md) -- self-balancing robot simulation model. Source repository can be found here: https://github.com/robustify/teeterbot
