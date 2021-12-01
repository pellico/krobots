from ktanks.tank_pb2 import *
import socket
from time import sleep
from multiprocessing import Process
from math import radians
class Comm:
    
    def __init__(self,name:str,server_ip : str,port:int) -> None:
        self.server_register_port = port
        self.server_ip = server_ip
        self.txSocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        #self.txSocket.settimeout(0)
        register_tank_command = RegisterTank()
        register_tank_command.name = name
        data = register_tank_command.SerializeToString()
        self.txSocket.sendto(data,(self.server_ip,self.server_register_port))
        answer, addr = self.txSocket.recvfrom(2048)
        self.port = addr[1]
        print(addr)
        self.txSocket.connect(addr)
        result = CommandResult()
        result.ParseFromString(answer)
        print(result.tick)
        #wait for start packet
        self.txSocket.recvfrom(2048)


    def send_receive_command(self,data):
        self.txSocket.send(data)
        print("Data sent")
        answer= self.txSocket.recv(2048)
        return answer

    def get_status(self):
        command=Command()
        command.command=Command.CommandId.GET_STATUS
        command.argument1 = 0.0
        command.argument2 = 0.0
        result =TankStatus()
        self._command_receive(command,result)
        return result

    def set_engine_power(self,fraction):
        command=Command()
        command.command=Command.CommandId.SET_ENGINE_POWER
        command.argument1 = float(fraction)
        command.argument2 = 0.0
        result =CommandResult()
        self._command_receive(command,result)
        return result

    def set_turning_impulse(self,fraction:float):
        command=Command()
        command.command=Command.CommandId.SET_TURNING_IMPULSE
        command.argument1 = float(fraction)
        command.argument2 = 0.0
        result =CommandResult()
        self._command_receive(command,result)
        return result
    
    def set_cannon_position(self,angle:float):
        command=Command()
        command.command=Command.CommandId.SET_CANNON_POSITION
        command.argument1 = float(angle)
        command.argument2 = 0.0
        result =CommandResult()
        self._command_receive(command,result)
        return result

    def get_radar_result(self,angle_increment:float,width:float):
        command=Command()
        command.command=Command.CommandId.GET_RADAR_RESULT
        command.argument1 = float(angle_increment)
        command.argument2 = float(width)
        result =RadarResult()
        self._command_receive(command,result)
        return result
    
    def fire_cannon(self):
        command=Command()
        command.command=Command.CommandId.FIRE_CANNON
        command.argument1 = 0.0
        command.argument2 = 0.0
        result =CommandResult()
        self._command_receive(command,result)
        return result
        
    
    def _command_receive(self,command,expected_answer):
        data = command.SerializeToString()
        while True:
            try:
                self.txSocket.send(data)
                answer= self.txSocket.recv(2048)
                expected_answer.ParseFromString(answer)
                break
            except Exception as e:
                print(e)

def set_angle(comm,angle:float,error:float):
    status=comm.get_status()
    if status.angle < angle +error and status.angle > angle - error:
        return
    comm.set_turning_impulse(0.01)
    while True:
        status=comm.get_status()
        if status.angle < angle +error and status.angle > angle - error:
            comm.set_turning_impulse(-0.01)
            comm.set_turning_impulse(0.0)
            return

def run_my_robot(name):
    comm = Comm(name,"127.0.0.1",55230)
    if name == "oreste":
        status = comm.get_status()
        status=comm.set_engine_power(0.0)
        #set_angle(comm,radians(180.0),0.01)
        print(status)
        #status=comm.set_engine_power(1.0)
        #print(status)
        for x in range(0,18) :
            comm.get_radar_result(radians(10),radians(10))
        comm.set_turning_impulse(0.1)    
        while True:
            status = comm.get_radar_result(radians(0),radians(10))
            if status.tanks :
                print (status)
        comm.set_engine_power(0.0)
        while True:
            sleep(5)
            print(comm.get_status())


    else:
        angle = 0;
        status=comm.set_engine_power(0.0)
        while True :
            sleep(1)
            print(comm.set_cannon_position(radians(angle)))
            print(comm.fire_cannon())
            angle +=90
            




if __name__ == '__main__':
    t1 = Process(target=run_my_robot, args=('bob',))
    t1.start()
    sleep(1)
    t2 = Process(target=run_my_robot, args=('oreste',))
    t2.start()
