#!/usr/bin/python
# -*- coding: utf-8 -*-
import gym, gym_fastsim
import time
from controllers.forward import ForwardController
from controllers.follow_wall import Follow_wallController


class SimEnv():
    
    def __init__(self, env, sleep_time, display):
        self.env = gym.make(env)
        self.env.reset()
        self.sleep_time = sleep_time
        self.display = display
        if(self.display):
            self.env.enable_display()

        self.obs, self.rew, self.done, self.info = self.env.step([0,0])


    def mouvement(self, c, n=1):
        for _ in range(n):
            obs, rew, done, info = self.env.step(c)
            print("Valeur de obs :" + str(obs))
            #print("Valeur de rew :" + str(rew))
            #print("Valeur de done :" + str(done))
            #print("Valeur de info :" + str(info))
            if(self.display):
                time.sleep(self.sleep_time)
            if self.done:
                break
            self.env.render()
            self.i+= 1
            
        return obs, rew, done, info

    def start(self):
        
        # initialize controllers
        forward = ForwardController(self.env, verbose=True)
        wall = Follow_wallController(self.env, verbose=True)
        self.controller = wall
        
        # start timers
        then = time.time()
        self.i = 0
        
        while not self.done:
            command = self.controller.get_command()
            self.obs, self.rew, self.done, self.info = self.mouvement(command)
            self.controller.reset()
            
        now = time.time()
        print("%d timesteps took %f seconds" % (self.i, now - then))
        input("Press Enter to continue...")
        self.env.close()        
    
    
if __name__=="__main__":
    env1 = 'kitchen-v1'
    env2 = 'maze-v0'
    sleep_time = 0.01
    display = True
    simEnv = SimEnv(env1, sleep_time, display)
    simEnv.start()
    
