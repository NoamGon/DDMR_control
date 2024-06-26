import NatNetClient as natnet
import numpy as np

def recieve_new_frame():
	print("newwww")
	
def recieve_marker_frame():
	print("markerrrr")


client = natnet.NatNetClient()
client.newFrameListener = recieve_new_frame
client.markerListener = recieve_marker_frame

server_ip = "192.168.0.162"
local_ip = "192.168.0.181"

#client.initialize(server_ip, local_ip)



try:
	while True:
		#sz_command="TimelinePlay"
		#return_code = client.send_command(sz_command)
		#print("Command: %s - return_code: %d"% (sz_command, return_code) )
		client.send_request(client.command_socket, client.NAT_REQUEST_MODELDEF,    "",  (client.server_ip_address, client.command_port) )
		
except KeyboardInterrupt:
	print("exit")
	
finally:
	client.shutdown()
	
