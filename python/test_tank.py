from ktanks.tank_pb2 import *
import socket
from time import sleep


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


    def send_receive_command(self,data):
        self.txSocket.send(data)
        print("Data sent")
        answer= self.txSocket.recv(2048)
        return answer

    def get_status(self):
        command=Command()
        command.command=Command.CommandId.GET_STATUS
        command.argument1 = 0.0
        result =TankStatus()
        self._command_receive(command,result)
        return result

    def set_engine_power(self,fraction):
        command=Command()
        command.command=Command.CommandId.SET_ENGINE_POWER
        command.argument1 = float(fraction)
        result =CommandResult()
        self._command_receive(command,result)
        return result

    def set_turning_impulse(self,fraction):
        command=Command()
        command.command=Command.CommandId.SET_TURNING_SPEED
        command.argument1 = float(fraction)
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

if __name__ == '__main__':
    comm = Comm("oreste","127.0.0.1",55230)
    while True:
        status = comm.get_status()
        print(status)
        status=comm.set_engine_power(1.0)
        print(status)
        status = comm.set_turning_impulse(1.0)
        sleep(1)
        
    

   


