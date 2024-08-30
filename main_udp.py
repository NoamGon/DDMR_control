
from socket import *
import math
import time
import matplotlib.pyplot as plt
import xm430_w210_motor_funcs_test as f
import sys
#import struct
import writeCSV
from datetime import datetime


clientSocket = socket(AF_INET, SOCK_DGRAM)
clientSocket.settimeout(5)

serveraddress = ("192.168.0.181", 5000)
clientSocket.bind(serveraddress)
				
# Generate a unique filename based on the current date and time
timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
csv_file1 = f'referance_{timestamp}.csv'
csv_file2 = f'robot_{timestamp}.csv'
# Write the header
writeCSV.write_to_csv(csv_file1, ['time', 'x_r', 'y_r', 'theta_r', 'x', 'y', 'theta', 'v', 'omega', 'x_e', 'y_e', "theta_e"], mode='w')

###### Constants ######
MOTOR_LIST = [1,2,3,4]
k_x = 0.9
k_y = 8
k_theta = 3
d = 0
L = 0.28  # m
R = 0.08/2
x = []
y = []
theta = []
x_desired = []
y_desired = []
OPERATING_MODE_V = 1    #velocity control
OPERATING_MODE_P = 4    #position control
OPERATING_MODE_I = 0    #current control

# 0 - current, 1 - velocity.
operating_mode = OPERATING_MODE_V

factor = 1.35
theta_e = 0

f.port_init()     


for motor in MOTOR_LIST:
    if motor in [1,2]:
        f.motor_init(motor,OPERATING_MODE_P)
    elif motor in [3,4]:
        f.motor_init(motor,operating_mode)
    else:
        sys.exit()

def get_trajectory_linear(time):
    global x_r, y_r, v_r, omega_r, radius, theta_r
    radius = 100000    #inf radius
    x_r = 10*time/100    #[m]
    y_r = 0 
    x_dot_r = 10*1/100     #10[cm/sec]
    y_dot_r = 0
    x_ddot_r = 0
    y_ddot_r = 0
    theta_r = 0
    v_r = math.sqrt(math.pow(x_dot_r, 2) + math.pow(y_dot_r, 2))
    omega_r = (y_ddot_r * x_dot_r - x_ddot_r * y_dot_r) / (math.pow(v_r, 2) +0.00001)
    x_desired.append(x_r)
    y_desired.append(y_r)
    
def get_trajectory_angular(time, motion_time):
    global x_r, y_r, v_r, omega_r, radius, theta_r
    radius = 1.4
    a = 0.5 * math.pi / motion_time
    x_r = radius*math.cos(a * time)
    y_r = radius*math.sin(a * time)
    x_dot_r = radius * a * math.sin(a * time)
    y_dot_r = -radius * a * math.cos(a * time)
    x_ddot_r = -radius * a * a * math.cos(a * time)
    y_ddot_r = -radius * a * a * math.sin(a * time)
    v_r = math.sqrt(math.pow(x_dot_r, 2) + math.pow(y_dot_r, 2))       
    omega_r = (y_ddot_r * x_dot_r - x_ddot_r * y_dot_r) / (math.pow(v_r, 2) +0.00001)
    x_desired.append(x_r)
    y_desired.append(y_r)
    theta_r = math.atan2(y_r,x_r+0.0001) + math.pi/2

def calculate_v(theta_e, x_e):
    return v_r * math.cos(theta_e) + k_x * x_e

def calculate_omega(y_e, theta_e):
    return omega_r + v_r * (k_y * y_e + k_theta * math.sin(theta_e))

def speed_to_cmd(speed): #rpm
    print('speed to cmd: ' + str(int(speed / (0.229 * factor * (2 * math.pi / 60)))))
    return int(speed / (0.229 * factor * (2 * math.pi / 60)))

def pos_to_cmd(pos):    #steering command
    command = int(pos * 180 / math.pi)
    command_norm = command % 360
    if command_norm > 180:
        command_norm -= 360
    if command_norm > 90:
        command_norm -= 180
    elif command_norm < -90:
        command_norm += 180
    if command_norm > 80:
        command_norm = 80
    elif command_norm < -80:
        command_norm = -80
    return int(command_norm / 0.088 )  

def torque_to_cmd(T):
    return int(((0.85 * T + 0.13) / 1000) / 2.69)

def low_level_controller_velcmd(v,omega):     
    # drive
    omega_r = L*omega/(2*R) + v/R   
    omega_l = -L*omega/(2*R) + v/R
    motor_cmd1 = speed_to_cmd(omega_r)
    motor_cmd2 = speed_to_cmd(omega_l)
    f.motor_execute(3, -motor_cmd2,OPERATING_MODE_V)
    f.motor_execute(4, motor_cmd1,OPERATING_MODE_V)

    #steering
    r = radius
    kp_theta = 1 
    kp_x = 0.2 
    kp_y = 0.8 
    lw = 0.19    #distance between wheels
    delta_r = math.atan(L/(r+0.5*lw))
    delta_l = math.atan(L/(r-0.5*lw))
    angle_command_r = delta_r - (kp_theta * theta_e + kp_y * y_e + kp_x * x_e)
    angle_command_l = delta_l - (kp_theta * theta_e + kp_y * y_e + kp_x * x_e)

    motor_cmd3 = pos_to_cmd(angle_command_r)          
    motor_cmd4 = pos_to_cmd(angle_command_l)          
    f.motor_execute(1, motor_cmd3,OPERATING_MODE_P)
    f.motor_execute(2, motor_cmd4,OPERATING_MODE_P)


def low_level_controller_currcmd(v,omega):
    # drive
    omega_r_ref = L*omega/(2*R) + v/R   
    omega_l_ref = -L*omega/(2*R) + v/R
    kp = 10
    omega_r_present = f.get_present_velocity(3)
    omega_l_present = f.get_present_velocity(4)
    error_r = omega_r_ref - omega_r_present
    error_l = omega_l_ref - omega_l_present
    torque_r = kp * error_r
    torque_l = kp * error_l
    f.motor_execute(3,torque_to_cmd(torque_r),OPERATING_MODE_I)
    f.motor_execute(4, torque_to_cmd(torque_l),OPERATING_MODE_I)

    #steering
    r = radius
    kp_theta = 1 
    kp_x = 0.2 
    kp_y = 0.8
    lw = 0.19    #distance between wheels
    delta_r = math.atan(L/(r+0.5*lw))
    delta_l = math.atan(L/(r-0.5*lw))
    angle_command_r = delta_r - (kp_theta * theta_e + kp_y * y_e + kp_x * x_e)
    angle_command_l = delta_l - (kp_theta * theta_e + kp_y * y_e + kp_x * x_e)

    motor_cmd3 = pos_to_cmd(angle_command_r)          
    motor_cmd4 = pos_to_cmd(angle_command_l)  
    f.motor_execute(1, motor_cmd3,OPERATING_MODE_P)
    f.motor_execute(2, motor_cmd4,OPERATING_MODE_P)


if __name__ == '__main__':
    ##### Initial Conditions #####
    v_r_initial = 0
    omega_r_initial = 0
    initial_time = time.time()
    relative_time = 0
    motion_time = 50
    loop_dt = 0.001

    try:
        while relative_time < motion_time:
            data, server = clientSocket.recvfrom(48)
            if len(data) != 0:
                de = data.decode("utf-8").split(',')
                y = int(de[0])/100
                x = int(de[1])/100
                theta = int(de[2])/100
                print("x: ",x," y: ",y, " theta: ",theta)
                #get_trajectory_linear(relative_time)
                get_trajectory_angular(motion_time-relative_time, motion_time)
                x_e = x_r - x
                y_e = y_r - y
                theta_e = theta_r - theta
                omega = calculate_omega(y_e, theta_e)
                v = calculate_v(theta_e, x_e)
                writeCSV.write_to_csv(csv_file1, [relative_time, x_r, y_r, theta_r, x, y, theta, v, omega, x_e, y_e, theta_e], mode='a')

                if operating_mode:
                    low_level_controller_velcmd(v,omega)
                else:
                    low_level_controller_currcmd(v, omega)
                    
                time.sleep(loop_dt)
                relative_time = time.time() - initial_time
      
    except KeyboardInterrupt:
        print('exit')
    finally:
        clientSocket.close()
        f.motor_execute(3, 0,OPERATING_MODE_V)
        f.motor_execute(4, 0,OPERATING_MODE_V)
        for i in MOTOR_LIST:
            f.motor_end(i)
