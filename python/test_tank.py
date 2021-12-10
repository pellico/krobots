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
            self.angimp_set = 0.05*delta_ang - 0.05*self.status.angvel
            self.comm.set_engine_power(self.forward_power,self.angimp_set)
            if abs(delta_ang) < error:
                self.comm.set_engine_power(self.forward_power,0.0)
                return
    
    def set_power_engine(self,power_fraction):
        self.forward_power=power_fraction
        self.comm.set_engine_power(self.forward_power,self.angimp_set)     

    def track_and_fire(self,step):
        positive_direction = True
        step_without_detection = 0
        while True:
            if positive_direction:
                radar_result = self.comm.get_radar_result(step,step)
            else:
                radar_result = self.comm.get_radar_result(-step,step)
            self.comm.set_cannon_position(radar_result.angle)
            if radar_result.tanks:
                self.comm.fire_cannon()
                step_without_detection =0
            else:
                if step_without_detection == 0:
                    positive_direction = not positive_direction
                elif step_without_detection > 10:
                    yield False 
                step_without_detection +=1
            yield True


def run_my_robot(name):
    tank= Tank(name)
    # my_angle=0
    # while True :
    #     my_angle+=90
    #     #tank.comm.set_cannon_position(radians(my_angle))
    #     sleep(0.5)
    #     tank.comm.fire_cannon()
    
    status = tank.comm.get_status()
    print(status)
    angle=radians(180)+status.angle
    print(angle+status.angle)
    tank.move_to_angle(angle,0.01)
    tank.set_power_engine(0.05)
    while True :
        radar_result = tank.comm.get_radar_result(0.1,0.1)
        if radar_result.tanks:
            tank.set_power_engine(0.01)
            generator = tank.track_and_fire(0.05)
            while next(generator):
                pass
        
            
    tank.set_power_engine(0.01)
    tank.comm.set_cannon_position(radar_result.angle)
    while True:
        tank.comm.fire_cannon()
    sleep(1)

            




if __name__ == '__main__':
    for tank_id in range(0,2):
        t2 = Process(target=run_my_robot, args=('oreste_%d' % (tank_id) ,))
        t2.start()
