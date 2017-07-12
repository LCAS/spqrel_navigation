#!/usr/bin/env python

from naoqi import ALProxy

# create proxy on ALMemory
memProxy = ALProxy("ALMemory","10.82.0.81",9559)
#raise event. Data can be int, float, list, string
memProxy.raiseEvent("NAOqiPlanner/Goal", [-7.29, -6.0])
