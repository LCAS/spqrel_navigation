#!/usr/bin/env python

from naoqi import *
import time
check = 0


# create python module
class myModule(ALModule):
  """python class myModule test auto documentation : comment needed to create a new python module"""


  def datachanged(self, strVarName, value):
    """callback when data change"""
    print "datachanged", strVarName, " ", value, " "
    global check
    check = 1

  def _PrivateMethod(self, param1, param2, param3):
    global check


broker = ALBroker("pythonBroker","0.0.0.0",0,"10.82.0.81",9559)


# call method
try:

  pythonModule2 = myModule("pythonModule2")
  prox = ALProxy("ALMemory")
  #prox.insertData("val",1) # forbiden, data is optimized and doesn't manage callback
  prox.subscribeToEvent("TopologicalNav/Status","pythonModule2", "datachanged") #  event is case sensitive !

except Exception,e:
  print "error"
  print e
  exit(1)

while (1):
  time.sleep(2)
