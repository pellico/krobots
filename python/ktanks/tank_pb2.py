# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: tank.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\ntank.proto\x12\x10protobuffer.tank\"\x1e\n\x06Vector\x12\t\n\x01x\x18\x01 \x01(\x02\x12\t\n\x01y\x18\x02 \x01(\x02\"\x8b\x01\n\nTankStatus\x12\x0c\n\x04tick\x18\x01 \x01(\r\x12*\n\x08velocity\x18\x02 \x01(\x0b\x32\x18.protobuffer.tank.Vector\x12\r\n\x05\x61ngle\x18\x03 \x01(\x02\x12\x14\n\x0c\x63\x61nnon_angle\x18\x04 \x01(\x02\x12\x0e\n\x06\x64\x61mage\x18\x05 \x01(\x02\x12\x0e\n\x06\x65nergy\x18\x06 \x01(\x02\"-\n\tTankRadar\x12\x10\n\x08\x64istance\x18\x01 \x01(\x02\x12\x0e\n\x06\x64\x61mage\x18\x02 \x01(\x02\"V\n\x0bRadarResult\x12\x0c\n\x04tick\x18\x01 \x01(\r\x12\r\n\x05\x61ngle\x18\x02 \x01(\x02\x12*\n\x05tanks\x18\x04 \x03(\x0b\x32\x1b.protobuffer.tank.TankRadar\".\n\rCommandResult\x12\x0c\n\x04tick\x18\x01 \x01(\r\x12\x0f\n\x07success\x18\x02 \x01(\x08\"\xe7\x01\n\x07\x43ommand\x12\r\n\x05index\x18\x01 \x01(\x07\x12\x34\n\x07\x63ommand\x18\x02 \x01(\x0e\x32#.protobuffer.tank.Command.CommandId\x12\x11\n\targument1\x18\x03 \x01(\x02\x12\x11\n\targument2\x18\x04 \x01(\x02\"q\n\tCommandId\x12\x0e\n\nGET_STATUS\x10\x00\x12\x14\n\x10GET_RADAR_RESULT\x10\x01\x12\x14\n\x10SET_ENGINE_POWER\x10\x02\x12\x0f\n\x0b\x46IRE_CANNON\x10\x03\x12\x17\n\x13SET_CANNON_POSITION\x10\x04\"\x1c\n\x0cRegisterTank\x12\x0c\n\x04name\x18\x01 \x01(\tb\x06proto3')



_VECTOR = DESCRIPTOR.message_types_by_name['Vector']
_TANKSTATUS = DESCRIPTOR.message_types_by_name['TankStatus']
_TANKRADAR = DESCRIPTOR.message_types_by_name['TankRadar']
_RADARRESULT = DESCRIPTOR.message_types_by_name['RadarResult']
_COMMANDRESULT = DESCRIPTOR.message_types_by_name['CommandResult']
_COMMAND = DESCRIPTOR.message_types_by_name['Command']
_REGISTERTANK = DESCRIPTOR.message_types_by_name['RegisterTank']
_COMMAND_COMMANDID = _COMMAND.enum_types_by_name['CommandId']
Vector = _reflection.GeneratedProtocolMessageType('Vector', (_message.Message,), {
  'DESCRIPTOR' : _VECTOR,
  '__module__' : 'tank_pb2'
  # @@protoc_insertion_point(class_scope:protobuffer.tank.Vector)
  })
_sym_db.RegisterMessage(Vector)

TankStatus = _reflection.GeneratedProtocolMessageType('TankStatus', (_message.Message,), {
  'DESCRIPTOR' : _TANKSTATUS,
  '__module__' : 'tank_pb2'
  # @@protoc_insertion_point(class_scope:protobuffer.tank.TankStatus)
  })
_sym_db.RegisterMessage(TankStatus)

TankRadar = _reflection.GeneratedProtocolMessageType('TankRadar', (_message.Message,), {
  'DESCRIPTOR' : _TANKRADAR,
  '__module__' : 'tank_pb2'
  # @@protoc_insertion_point(class_scope:protobuffer.tank.TankRadar)
  })
_sym_db.RegisterMessage(TankRadar)

RadarResult = _reflection.GeneratedProtocolMessageType('RadarResult', (_message.Message,), {
  'DESCRIPTOR' : _RADARRESULT,
  '__module__' : 'tank_pb2'
  # @@protoc_insertion_point(class_scope:protobuffer.tank.RadarResult)
  })
_sym_db.RegisterMessage(RadarResult)

CommandResult = _reflection.GeneratedProtocolMessageType('CommandResult', (_message.Message,), {
  'DESCRIPTOR' : _COMMANDRESULT,
  '__module__' : 'tank_pb2'
  # @@protoc_insertion_point(class_scope:protobuffer.tank.CommandResult)
  })
_sym_db.RegisterMessage(CommandResult)

Command = _reflection.GeneratedProtocolMessageType('Command', (_message.Message,), {
  'DESCRIPTOR' : _COMMAND,
  '__module__' : 'tank_pb2'
  # @@protoc_insertion_point(class_scope:protobuffer.tank.Command)
  })
_sym_db.RegisterMessage(Command)

RegisterTank = _reflection.GeneratedProtocolMessageType('RegisterTank', (_message.Message,), {
  'DESCRIPTOR' : _REGISTERTANK,
  '__module__' : 'tank_pb2'
  # @@protoc_insertion_point(class_scope:protobuffer.tank.RegisterTank)
  })
_sym_db.RegisterMessage(RegisterTank)

if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _VECTOR._serialized_start=32
  _VECTOR._serialized_end=62
  _TANKSTATUS._serialized_start=65
  _TANKSTATUS._serialized_end=204
  _TANKRADAR._serialized_start=206
  _TANKRADAR._serialized_end=251
  _RADARRESULT._serialized_start=253
  _RADARRESULT._serialized_end=339
  _COMMANDRESULT._serialized_start=341
  _COMMANDRESULT._serialized_end=387
  _COMMAND._serialized_start=390
  _COMMAND._serialized_end=621
  _COMMAND_COMMANDID._serialized_start=508
  _COMMAND_COMMANDID._serialized_end=621
  _REGISTERTANK._serialized_start=623
  _REGISTERTANK._serialized_end=651
# @@protoc_insertion_point(module_scope)
