# Two Wheel Balancer


## Running application

To run application try one of:
- run *balancer-driver/src/drive*
- run *balancer-driver/src/balancer/main.py* 
- execute *cd balancer-driver/src; python3 -m balancer*


### Running tests

To run tests execute *src/runtests.py*. It can be run with code profiling 
and code coverage options.


### rqt plugin

Running:
- ```rqt -vvv --standalone rqt_balancer```
- ```rosrun rqt_balancer rqt_balancer```
- ```rqt```

Plugins list:
```rqt --list-plugins```

Packages list:
```rospack list```

Fix/detect:
```rqt --force-discover```


## Running system

- start core: ```roscore```
- launch Gazebo with robot world ```roslaunch teeterbot_gazebo teeterbot_empty_world.launch```
- start *balancer* driver
- start *rqt* GUI
- run *balancer-driver/plot*.sh* to observer inputs and outputs


## Required libraries

- *numpy-quaternion*
- *matplotlib*


# References

- http://iasj.net/iasj?func=fulltext&aId=143863
- http://wiki.ros.org/rqt/Tutorials/Create%20your%20new%20rqt%20plugin
- http://wiki.ros.org/rqt/Tutorials/Writing%20a%20Python%20Plugin
- https://pythonhosted.org/scikit-fuzzy/index.html

