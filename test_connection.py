import socket
from time import sleep
portNum = 55230


txSocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
txSocket.sendto("hello".encode(),("127.0.0.1",portNum))
print("Data sent")
data, addr = txSocket.recvfrom(2048)
print((data,addr))

