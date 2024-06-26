from socket import *
#import struct

clientSocket = socket(AF_INET, SOCK_DGRAM)
clientSocket.settimeout(5)

serveraddress = ("192.168.0.181", 5000)
clientSocket.bind(serveraddress)
try:
	while True:
		#clientSocket.sendto(b"", serveraddress)
		data, server = clientSocket.recvfrom(48)
		if len(data)!=0:
			
		#print(type(data))
			print(data.decode("utf-8"))
			de = data.decode("utf-8").split(',')
			print(de)
			x = int(de[0])
			z = int(de[1])
			print("x: ",x," z: ",z)
		#data = struct.unpack("f",data)
		#print(data)
		
except KeyboardInterrupt:
	print("exit")
	
finally:
	clientSocket.close()
