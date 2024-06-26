import sys
import time
from NatNetClient import NatNetClient
import DataDescriptions
import MoCapData
import numpy as np
#import transforms3d.euler as euler

#sys.path.append('/home/dano/Downloads/diff_drive_main')
#import xm430_w210_motor_funcs as f
#import diff_algorithm as da


xy_factor = 40
speed_factor = 100


'''/// MAIN ALGORITHM ///'''

def receive_rigid_body_frame( new_id, position, rotation ):
    Vc = 2
    
    # Convert quaternion to Euler angles (in radians)
    #euler_angles = euler.quat2euler(rotation, 'sxyz')
    # Print the results
    print("Euler angles (XYZ):", rotation)
    print(position)
    #theta = euler_angles[0]
    
    #controll = da.diff_ctrl([-xy_factor*position[1],xy_factor*position[0], theta], Vc)
    
    #f.motor_execute(3, -int(speed_factor*controll[0]))
    #f.motor_execute(2, int(speed_factor*controll[1]))
    
    
    if (xy_factor*position[0])**2+(xy_factor*position[1])**2 > 360:
        #f.motor_execute(3, 0)
        #f.motor_execute(2, 0)
        sys.exit()
        

'''//////////////////////'''

  
def my_parse_args(arg_list, args_dict):
    # set up base values
    arg_list_len=len(arg_list)
    if arg_list_len>1:
        args_dict["serverAddress"] = arg_list[1]
        if arg_list_len>2:
            args_dict["clientAddress"] = arg_list[2]
        if arg_list_len>3:
            if len(arg_list[3]):
                args_dict["use_multicast"] = True
                if arg_list[3][0].upper() == "U":
                    args_dict["use_multicast"] = False

    return args_dict

optionsDict = {}
optionsDict["clientAddress"] = "192.168.0.181"
optionsDict["serverAddress"] = "192.168.0.162"
optionsDict["use_multicast"] = True

# This will create a new NatNet client
optionsDict = my_parse_args(sys.argv, optionsDict)

streaming_client = NatNetClient()
streaming_client.set_client_address(optionsDict["clientAddress"])
streaming_client.set_server_address(optionsDict["serverAddress"])
streaming_client.set_use_multicast(optionsDict["use_multicast"])

streaming_client.run()
streaming_client.connected()


# Configure the streaming client to call our rigid body handler on the emulator to send data out.
streaming_client.rigid_body_listener = receive_rigid_body_frame


    
    
