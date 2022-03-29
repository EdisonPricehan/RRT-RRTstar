# RRT-RRTstar
Implementation and visualization of RRT and RRTstar algorithms

## Run python scripts for each algorithm

`python3 ./RRT.py`

`python3 ./RRT_STAR.py`

Double click once to select start node, twice to select goal node. 

Shut down and re-run the script for another test.

## Tuning

* 4 example images are provided for test, you can specify which image to use in RRT/RRTstar class param "map_name".

* Start and goal nodes can also be hard coded and passed to RRT/RRTstar class.

* Maximum iteration (default 10000) and step size (default 10) can also be tuned.

* Radius param for RRTstar class is to specify the circular region around new random node where nodes' shortest cost and parent may be updated.

* Try_goal param being enabled will speed up goal search, which terminates if new random node can connect to goal with no collision.

* Path smoothing is by default enabled, and the smoothened path will show in a different color with the raw path.

## Thanks to

https://gist.github.com/Fnjn/58e5eaa27a3dc004c3526ea82a92de80

https://github.com/nimRobotics/RRT
