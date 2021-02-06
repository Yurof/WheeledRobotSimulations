# -*- coding: utf-8 -*-
"""
Created on Fri Jan 15 11:16:01 2021

@author: Youssef
"""
import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
#p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [1,1,0]

boxId = p.loadURDF("kitchen.urdf",[0, 0,0])
boxId = p.loadURDF("robot.urdf",startPos)
#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()
