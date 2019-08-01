# Two Wheeled Balancer

Project contains two modules:
- *balancer-driver* containing exmaples of control algorithms
- *balancer-rqt* containig example of GUI made for *rqt*


## Running application

To run application try one of:
- run *balancer-driver/src/drive*
- run *balancer-driver/src/balancer/main.py* 
- execute *cd balancer-driver/src; python3 -m balancer*


## Running tests

To run tests execute *balancer-driver/src/runtests.py*. It can be run with code profiling 
and code coverage options.


## *rqt* options:

To run *balancer* plugin for *rqt* execute one of following commands:
- ```rqt -vvv --standalone rqt_balancer```
- ```rosrun rqt_balancer rqt_balancer```
- ```rqt```

Plugins list:
```rqt --list-plugins```

Fix/detect plugin:
```rqt --force-discover```

Packages list:
```rospack list```


# Required libraries

- *numpy-quaternion*
- *matplotlib*


# References

- http://iasj.net/iasj?func=fulltext&aId=143863
- http://wiki.ros.org/rqt/Tutorials/Create%20your%20new%20rqt%20plugin
- http://wiki.ros.org/rqt/Tutorials/Writing%20a%20Python%20Plugin
- https://pythonhosted.org/scikit-fuzzy/index.html

