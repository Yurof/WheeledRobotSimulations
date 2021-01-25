#!/usr/bin/python
# -*- coding: utf-8 -*-

""" Alexandre Coninx
    ISIR CNRS/UPMC
    04/06/2019
""" 

# Port of the Fastsim example


from pyfastsim import *
import sys
import time
import random

if(len(sys.argv) < 2):
	print("This program cannot run: you need to provide a XML file (e.g.,%s worlds/example.xml)" % sys.argv[0])
	sys.exit(1)


settings = Settings(sys.argv[1])

env_map = settings.map()
robot = settings.robot()
radar = robot.get_radars()[0]
goal = env_map.get_goals()[0]

d = Display(env_map, robot)

i=0
while (abs(goal.get_x()-env_map.real_to_pixel(robot.get_pos().x()))>5 or abs(goal.get_y()-env_map.real_to_pixel(robot.get_pos().y()))>5) and i<100000:
	r1 = random.randint(-2,5)
	r2 = random.randint(-2,5)
	#d.update()


	pos = robot.get_pos()
	#print("Step %d robot pos: x = %f x = %f   y = %f y = %f  theta = %f slide =%d" % (i, pos.x(),env_map.real_to_pixel(robot.get_pos().x()), pos.y(),env_map.real_to_pixel(robot.get_pos().y()), pos.theta(), robot.get_radars()[0].get_activated_slice()))
	
	robot.move(r1, r2, env_map)

	if robot.get_collision()==True:
		robot.reinit()
		#print("collission ")
		robot.move(-0.5,0.5 , env_map)
		robot.move(0.5,0.5, env_map)
	#time.sleep(0.1)
	i+=1

print("Step %d"% (i))



