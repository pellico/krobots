'''
Created on 29 Apr 2022

@author: root

Simple regression test that compare stored tank status with status got the simulation.
It is expected that on same platform Windows10 64 bit we get the same result.

'''
import unittest, threading, pickle
from collections import namedtuple
from typing import Iterator

from subprocess import Popen

import ktanks

from time import sleep
from multiprocessing import Process
from math import radians, pi
from ktanks.tank_pb2 import TankStatus


def angle_wrapping(angle:float) -> float:
    angle_res = angle;
    while True: 
        if angle_res > pi:
            angle_res = angle_res - 2.0 * pi
        elif angle_res <= -pi:
            angle_res = angle_res + 2.0 * pi
        else:
            break
    return angle_res


class Tank:

    def __init__(self, name):
        self.name = name
        # self.comm = ktanks.Tank(name,"10.136.240.148",55230,use_tcp=True)

    def move_to_angle (self, angle, error):
        self.status = self.comm.get_status()
        yield self.status
        self.target_angle = angle
        delta_ang = angle - self.status.angle
        if abs(delta_ang) < error:
            return
        while True:
            self.status = self.comm.get_status()
            yield self.status
            delta_ang = angle_wrapping(angle - self.status.angle)
            self.angimp_set = 0.05 * abs(delta_ang) - 0.05 * abs(self.status.angvel)
            self.comm.set_engine_power(self.forward_power, self.angimp_set)
            if abs(delta_ang) < error:
                yield self.comm.set_engine_power(self.forward_power, 0.0)
                break
    
    def set_power_engine(self, power_fraction):
        self.forward_power = power_fraction
        yield self.comm.set_engine_power(self.forward_power, self.angimp_set)     

    def track_and_fire(self, step):
        positive_direction = True
        step_without_detection = 0
        while True:
            if positive_direction:
                radar_result = self.comm.get_radar_result(step, step)
            else:
                radar_result = self.comm.get_radar_result(-step, step)
            tank_status=self.comm.set_cannon_position(radar_result.angle)
            if radar_result.tanks:
                self.comm.fire_cannon()
                step_without_detection = 0
            else:
                if step_without_detection == 0:
                    positive_direction = not positive_direction
                elif step_without_detection > 10:
                    break
                step_without_detection += 1
            yield tank_status
            
    def stepper(self) -> Iterator[TankStatus]:
        self.comm = ktanks.Tank(self.name, "127.0.0.1", 55230)
        self.forward_power = 0.0
        self.target_angle = 0.0
        self.angimp_set = 0.0
        self.status = self.comm.get_status()
        status = self.comm.get_status()
        yield status
        yield from self.move_to_angle(status.power_source.p, 0.01)
        yield from self.set_power_engine(0.1)
        while True:
            yield from self.set_power_engine(0.1)
            radar_result = self.comm.get_radar_result(0.1, 0.1)
            if radar_result.tanks:
                yield from self.set_power_engine(0.01)
                yield from self.track_and_fire(0.01)


Status = namedtuple("Status", "tick velocity_r velocity_p angle cannon_angle damage "
                    "energy success angvel power_source_r power_source_p cannon_temp")


def status2Tuple(status):
    return Status(status.tick,status.velocity.r,status.velocity.p,status.angle,
                  status.cannon_angle,status.damage,status.energy,status.success,
                  status.angvel,status.power_source.r,status.power_source.p,status.cannon_temp)
           
comparison_done_barrier = threading.Barrier(3,timeout=120)
    
def run_tank(name, mode):
    num_steps = 7600
    tank = Tank(name)
    stepper = tank.stepper()
    file_name = f"regression_data/{name}.dat"
    try:
        if mode == "save":
            with open(file_name, "wb") as data_file: 
                for _ in range(num_steps):
                    status = next(stepper)
                    pickle.dump(status2Tuple(status), data_file, 5)
        else:
            with open(file_name, "rb") as data_file: 
                for _ in range(num_steps):
                    status = next(stepper)
                    data = pickle.load(data_file)
                    if status2Tuple(status) != data:
                        pass
                        #print ((name,data))
                        #print((name,status2Tuple(status)))
                        raise Exception(f"Failed test @tick {status.tick}")
    except ConnectionResetError:
    # This is OK becasue server shutdown if some client close TCP connection.
        pass
    

                    
    

class RegressionTests(unittest.TestCase):
    """
    Run simulation with two tanks in debug mode.
    Record status of both tanks at each step and compare with stored values.
    """

    def setUp(self):
        # Run server for 2 tank in debug mode
        self.server = Popen([r"..\target\debug\ktanks_server.exe", "2", "-d"])

    def tearDown(self):
        self.server.kill()

    def testName(self):
        t1 = threading.Thread(target=run_tank, args=("t1", "compare"))
        t1.start()
        sleep(2)
        t2 = threading.Thread(target=run_tank, args=("t2", "compare"))
        t2.start()
        t1.join()
        t2.join()




if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
