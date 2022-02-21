# krobots
# Copyright (C) 2021  Oreste Bernardi

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
from .tank_pb2 import *
import socket
from typing import Callable,Any,Union


from typing import TypeVar
# Workaroudn waiting for  PEP673
TTank = TypeVar("TTank", bound="Tank")
TResult = TypeVar("TResult",bound = Union[RadarResult,SimulationConfig,TankStatus])
class Tank:
    """
    Class to interface with simulation server
    """ 
        
    def __init__(self,name:str,server_ip : str,port:int,use_tcp:bool=True) -> None:
        """Create a tank at the server and control it.
        
        :param name: Name of tank
        :param server_ip: IP of simulation server
        :param port: Port of simulation server
        :param use_tcp: If false UDP protocol is used instead of TCP. It shall metch server setting
        
        """
        self.server_register_port = port
        self.server_ip = server_ip
        
        register_tank_command = RegisterTank()
        register_tank_command.name = name
        data = register_tank_command.SerializeToString()

        if use_tcp:
            self.txSocket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            self.txSocket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.txSocket.connect((self.server_ip,self.server_register_port))
            self.txSocket.settimeout(None)
            self.txSocket.send(data)
            answer, addr = self.txSocket.recvfrom(2048)
        else:
            self.txSocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            self.txSocket.sendto(data,(self.server_ip,self.server_register_port))
            answer, addr = self.txSocket.recvfrom(2048)
            self.txSocket.connect(addr)
        
        self.port = addr[1]
        #: Configuration of simulator
        self.simulation_configuration : SimulationConfig  = SimulationConfig() 
        self.simulation_configuration.ParseFromString(answer)
        #wait for start packet
        self.txSocket.recvfrom(2048)
        # Set timeout in order to allow python script kill by 
        # keyboard interrupt
        self.txSocket.settimeout(2)
        self._command_receive : Callable[[Command,TResult], Any]
        if self.simulation_configuration.debug_mode:
            #Enable debug mode. Client debug mode is compatible with any running mode
            #of game server.
            self._command_receive=self._command_receive_debug
        else:
            #Disable debug. If game server is running debug mode and some
            #client are stopped, client can fail with timeout 
            self._command_receive=self._command_receive_release
            
        
            

    def get_status(self)->TankStatus:
        """Get tank status
        
        """
        command=Command()
        command.command=Command.CommandId.GET_STATUS
        command.index = 0x12345 # This to avoid empty protobuf message. TCP cannot send empty messages.
        command.argument1 = 0.0
        command.argument2 = 0.0
        result =TankStatus()
        self._command_receive(command,result)
        return result

    def set_engine_power(self,fraction_forward_power:float,fraction_turning_power:float) -> TankStatus:
        """
        Set engine power
        
        :param fraction_forward_power: Power fraction applied to move forward or backward. [-1.0,1.0] Negative number move backward 
        :param fraction_turning_power: Power fraction applied to turn the tank. [-1.0,1.0] Negative number move counter-clokwise
    
        """
        command=Command()
        command.command=Command.CommandId.SET_ENGINE_POWER
        command.argument1 = float(fraction_forward_power)
        command.argument2 = float(fraction_turning_power)
        result =TankStatus()
        self._command_receive(command,result)
        return result
    
    def set_cannon_position(self,angle:float)->TankStatus:
        """
        Set cannon position
        
        :param angle: Angle relative to tank [radians]
        
        """
        command=Command()
        command.command=Command.CommandId.SET_CANNON_POSITION
        command.argument1 = float(angle)
        command.argument2 = 0.0
        result =TankStatus()
        self._command_receive(command,result)
        return result

    def get_radar_result(self,angle_increment:float,width:float)->RadarResult:
        """
        Move radar and get radar result
        
        :param angle_increment: How many radians shall be rotated the radar before sampling Max: :py:attr:.simulation_configuration
        :param width: Width of the radar in radians Max see :py:attr:.simulation_configuration
        
        """
        command=Command()
        command.command=Command.CommandId.GET_RADAR_RESULT
        command.argument1 = float(angle_increment)
        command.argument2 = float(width)
        result =RadarResult()
        self._command_receive(command,result)
        return result
    
    def fire_cannon(self)->TankStatus:
        """
        Fire cannon
        """
        command=Command()
        command.command=Command.CommandId.FIRE_CANNON
        command.argument1 = 0.0
        command.argument2 = 0.0
        result =TankStatus()
        self._command_receive(command,result)
        return result
        
    
    def _command_receive_release(self,command : Command,expected_answer : TResult):
        data = command.SerializeToString()
        self.txSocket.send(data)
        answer= self.txSocket.recv(2048)
        expected_answer.ParseFromString(answer)
    
    def _command_receive_debug(self,command:Command,expected_answer:TResult):
        data = command.SerializeToString()
        self.txSocket.send(data)
        while True:
            # This support server in debug mode. It doesn't raise an exception
            # if server is not answering within the timeout of 2 sec.
            # It has the same functionaliaty as Tank
            try:
                answer= self.txSocket.recv(2048)
            except socket.timeout:
                continue
            expected_answer.ParseFromString(answer)
            break
        
        
