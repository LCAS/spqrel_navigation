#!/usr/bin/env python
import sys
from naoqi import ALProxy
import time
# create proxy on ALMemory
memProxy = ALProxy("ALMemory","10.82.0.81",9559)

#raise event. Data can be int, float, list, string
memProxy.raiseEvent("TopologicalNav/GetPlan", sys.argv[1])
time.sleep(3)

route = memProxy.getData("TopologicalNav/Route")
print route