from NatNetClient import NatNetClient
import numpy as np

def receive_frame(frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount, labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged):
    print("Received frame")

def receive_rigid_body_data(rigid_body_id, position, rotation):
    print("Rigid Body ID:", rigid_body_id)
    print("Position:", position)
    print("Rotation:", rotation)

def receive_marker_set_data(marker_set_name, marker_positions):
    print(f"Marker Set Name: {marker_set_name}")
    for marker in marker_positions:
        x, y, z = marker
        print(f"Marker: x={x:.2f}, y={y:.2f}, z={z:.2f}")

# This is a callback function that gets connected to the NatNet client
def receive_unlabeled_marker_data(id, position):
    x, y, z = position
    print(f"Unlabeled Marker ID {id}: x={x:.2f}, y={y:.2f}, z={z:.2f}")

# Create a new client
client = NatNetClient()

# Configure the callbacks
#client.newFrameListener = receive_frame
#client.rigidBodyListener = receive_rigid_body_data
#client.markerSetListener = receive_marker_set_data
#client.unlabeledMarkerListener = receive_unlabeled_marker_data

# Connect the client

try:
		client.run()
		while True:
			#client.send_request(client.command_socket, client.NAT_REQUEST_MODELDEF,    "",  (client.server_ip_address, client.command_port) )
			bot = client.rigid_bodies["Bot"]
			print(bot.position)
except KeyboardInterrupt:
	print("exit")
	
finally:
	client.shutdown()
