from .tank_pb2 import *
import socket
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
        result = TankStatus()
        result.ParseFromString(answer)
        print(result.tick)
        #wait for start packet
        self.txSocket.recvfrom(2048)


    def send_receive_command(self,data):
        self.txSocket.send(data)
        answer= self.txSocket.recv(2048)
        return answer

    def get_status(self)->TankStatus:
        command=Command()
        command.command=Command.CommandId.GET_STATUS
        command.argument1 = 0.0
        command.argument2 = 0.0
        result =TankStatus()
        self._command_receive(command,result)
        return result

    def set_engine_power(self,fraction_forward_power:float,fraction_turning_power:float) -> TankStatus:
        command=Command()
        command.command=Command.CommandId.SET_ENGINE_POWER
        command.argument1 = float(fraction_forward_power)
        command.argument2 = float(fraction_turning_power)
        result =TankStatus()
        self._command_receive(command,result)
        return result
    
    def set_cannon_position(self,angle:float)->TankStatus:
        command=Command()
        command.command=Command.CommandId.SET_CANNON_POSITION
        command.argument1 = float(angle)
        command.argument2 = 0.0
        result =TankStatus()
        self._command_receive(command,result)
        return result

    def get_radar_result(self,angle_increment:float,width:float)->RadarResult:
        command=Command()
        command.command=Command.CommandId.GET_RADAR_RESULT
        command.argument1 = float(angle_increment)
        command.argument2 = float(width)
        result =RadarResult()
        self._command_receive(command,result)
        return result
    
    def fire_cannon(self)->RadarResult:
        command=Command()
        command.command=Command.CommandId.FIRE_CANNON
        command.argument1 = 0.0
        command.argument2 = 0.0
        result =TankStatus()
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