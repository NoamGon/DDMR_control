from dynamixel_sdk import *

# Control table addresses for XM430 motor
ADDR_OPERATING_MODE = 11       # Address for operating mode
ADDR_TORQUE_ENABLE = 64       # Address for torque enable
ADDR_GOAL_VELOCITY = 104       # Address for goal velocity
ADDR_PRESENT_VELOCITY = 128    # Address for present velocity
GOAL_POSITION = 116
# Operating mode values
OPERATING_MODE_VELOCITY = 1    # Value for velocity control mode

# Default settings
PROTOCOL_VERSION = 2.0         # Dynamixel protocol version
BAUDRATE = 57600               # Default baudrate of the Dynamixel XM430
DEVICE_NAME = '/dev/ttyUSB0'   # Serial port device name (may vary depending on your setup)



def port_init():
    # Initialize PortHandler and PacketHandler
    global port_handler
    global packet_handler
    port_handler = PortHandler(DEVICE_NAME)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    # Open the port
    if port_handler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        exit(1)

    # Set baudrate
    if port_handler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        exit(1)

def motor_init(dxl_id, operating_mode):
    # Set operating mode to velocity control
    global port_handler
    global packet_handler

    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_OPERATING_MODE, operating_mode)
    if dxl_comm_result != COMM_SUCCESS:
        print(packet_handler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print(packet_handler.getRxPacketError(dxl_error))
    else:
        print("Operating mode has been set to velocity control")

    # Enable torque
    torque_enable = 1  # 1 to enable torque, 0 to disable torque

    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE, torque_enable)
    if dxl_comm_result != COMM_SUCCESS:
        print(packet_handler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print(packet_handler.getRxPacketError(dxl_error))
    else:
        print("Torque enable has been set successfully")


'''/// START OF THE ALGORITHM ///'''
def motor_execute(motor_id, goal_value, operating_mode):
    global port_handler
    global packet_handler
    # if motor_id in [3,4,5,6]:
    # # Set goal velocity/current/position...
    #     dxl_comm_result, dxl_error = packet_handler.write4ByteTxRx(port_handler, motor_id, ADDR_GOAL_VELOCITY,
    #                                                            goal_value)    #unit: 0.229rpm, value between -1023 to 1023
    # elif motor_id in [1,2]:
    #     dxl_comm_result, dxl_error = packet_handler.write4ByteTxRx(port_handler, motor_id, GOAL_POSITION,
    #                                                            goal_value)    #unit: 0.088 [deg/pulse], 1[rev] : 0 ~ 4,095
    if operating_mode == 1:
        dxl_comm_result, dxl_error = packet_handler.write4ByteTxRx(port_handler, motor_id, ADDR_GOAL_VELOCITY,
                                                                goal_value)    #unit: 0.229rpm, value between -1023 to 1023
    elif operating_mode == 4:
        dxl_comm_result, dxl_error = packet_handler.write4ByteTxRx(port_handler, motor_id, GOAL_POSITION,
                                                               goal_value)    #unit: 0.088 [deg/pulse], 1[rev] : 0 ~ 4,095
    if dxl_comm_result != COMM_SUCCESS:
        print(packet_handler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print(packet_handler.getRxPacketError(dxl_error))
    else:
        print("Goal velocity has been set successfully")


def motor_end(motor_id):
    global port_handler
    global packet_handler

    # Disable torque and close the port
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, 1, ADDR_TORQUE_ENABLE, 0)
    if dxl_comm_result != COMM_SUCCESS:
        print(packet_handler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print(packet_handler.getRxPacketError(dxl_error))

    port_handler.closePort()

