# Two Wheel Balancer

## Running application

To run application try one of:
- run *src/drive*
- run *src/balancer/main.py* 
- execute *cd src; python3 -m balancer*


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


# References

- http://iasj.net/iasj?func=fulltext&aId=143863
- http://wiki.ros.org/rqt/Tutorials/Create%20your%20new%20rqt%20plugin
- http://wiki.ros.org/rqt/Tutorials/Writing%20a%20Python%20Plugin
- https://pythonhosted.org/scikit-fuzzy/index.html

