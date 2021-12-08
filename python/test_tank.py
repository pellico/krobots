from ktanks import *

from time import sleep
from multiprocessing import Process
from math import radians


class Tank:
    def __init__(self,name):
        self.name = name
        self.comm = Comm(name,"127.0.0.1",55230)
        self.forward_power= 0.0
        self.target_angle = 0.0
        self.angimp_set = 0.0
        self.status = self.comm.get_status()
    
    def move_to_angle (self,angle,error):
        self.status = self.comm.get_status()
        self.target_angle = angle
        delta_ang = angle - self.status.angle
        if abs(delta_ang) < error:
            return
        while True:
            self.status = self.comm.get_status()
            delta_ang = angle - self.status.angle
            self.angimp_set = 0.01*delta_ang - 0.02*self.status.angvel
            self.comm.set_engine_power(self.forward_power,self.angimp_set)
            if abs(delta_ang) < error:
                self.comm.set_engine_power(self.forward_power,0.0)
                return
         

def run_my_robot(name):
    tank= Tank("Oreste")
    
    status = tank.comm.get_status()
    print(status)
    angle=radians(90)
    print(angle)
    tank.move_to_angle(angle,0.01)
    sleep(1)

            




if __name__ == '__main__':
    for tank_id in range(0,1):
        t2 = Process(target=run_my_robot, args=('oreste_%d' % (tank_id) ,))
        t2.start()
