import socket

import numpy as np
import matplotlib.pyplot as plt
from pyproj import Proj, Transformer

from PID_autop import PID

# start FlightGear
# fgfs --generic=socket,out,10,127.0.0.1,5051,udp,control --generic=socket,in,10,127.0.0.1,5052,udp,control --httpd=5405 --disable-sound

# Socket connection
UDP_IP = "127.0.0.1"
UDP_Port_input = 5052
UDP_Port_output = 5051
flightgear_2_client = (UDP_IP, UDP_Port_output)
client_2_flightgear = (UDP_IP, UDP_Port_input)


def send_data(ip_and_port, data):
    # Connect to flightgear via socket to receive data
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data_bytes = data.encode('utf-8')
    send_sock.sendto(data_bytes, ip_and_port)
    #  print("Sent data:", data)
    send_sock.close()


def receive_data(sock_connection):
    # Send data to flightgear
    data, address = sock_connection.recvfrom(1024)
    #  print(f"Received data:{data.decode('utf-8').split()}\n")
    return data.decode('utf-8')


dt = 1

pid_flightpath = PID(0.035, 0.04, 0.001, 0.1, 0.5, -0.5)
pid_roll = PID(0.003, 0.01, 0.002, 0.1, 0.25, -0.25)
"""
t = np.arange(0, 2500, 5 * dt)
path = np.zeros((len(t), 2))
path[:, 0] = 30 * t
path[:, 1] = 50 * t
"""
"""
t = np.arange(0, 2500, 5 * dt)
omp = 0.0025
path = 12000 * np.column_stack((np.sin(omp * t), 1 - np.cos(omp * t)))
"""
t = np.arange(0, 1351, dt)
U0 = 50  # Adjust U0 as needed
omp = 0.0025

path = np.zeros((len(t), 2))
path[:, 0] = U0 * t
path[:, 1] = 500 * (-np.cos(2 * np.pi * omp * t) + 1) * np.exp(0.002 * t)


fig, ax = plt.subplots()
ax.set_title('2D Path')
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.grid(True)
previous_x_values = []
previous_y_values = []

wgs84 = Proj(init='epsg:4326')  # WGS84 coordinate system
initial_latitude_deg = None
initial_longitude_deg = None
while True:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_Port_output))
    raw_data_output = receive_data(sock)

    "TARGET POINTS"
    target_flightpath_angle = 0.0
    data_values = raw_data_output.split()

    roll_deg = float(data_values[0])
    pitch_deg = float(data_values[1])
    alpha_deg = float(data_values[2])

    altitiude_ft = float(data_values[4])
    latitude_deg = float(data_values[5])
    longitude_deg = float(data_values[6])
    """
        no_fts = float(data_values[7])
        ea_fts = float(data_values[8])
        do_fts = float(data_values[9])
        yd, xd = no_fts, ea_fts
        U0 = float(data_values[10])*1.68781
        L1 = 8000
        
        xd = float(data_values[11])
        yd = float(data_values[12]) 
        zd = float(data_values[13])
        U0 = float(data_values[10])*1.68781
        L1 = 10004
        """
    U0 = float(data_values[10]) * 1.68781
    roll_degg = float(data_values[14]) / (180 / 3.141592653589793)
    heading_degg = float(data_values[15]) / (180 / 3.141592653589793)
    v = float(data_values[12])
    xd = float(data_values[8])  # east
    yd = float(data_values[7])  # north
    #xd = U0*np.cos(heading_degg)-v*np.cos(roll_degg)*np.sin(heading_degg)
    #yd = U0*np.sin(heading_degg)+v*np.cos(roll_degg)*np.cos(heading_degg)
    L1 = 2000

    utm_zone = int((longitude_deg + 180) / 6) + 1  # Calculate the UTM zone
    utm_projection = Proj(proj='utm', zone=utm_zone, ellps='WGS84')
    transformer = Transformer.from_proj(wgs84, utm_projection, always_xy=True)
    if initial_latitude_deg is None or initial_longitude_deg is None:
        # Store the initial coordinates when they are not yet set
        initial_longitude_deg, initial_latitude_deg = transformer.transform(longitude_deg, latitude_deg)
        initial_latitude_deg = initial_latitude_deg - 0 * 1600
        initial_longitude_deg = initial_longitude_deg - 0 * 5000
    # Perform the coordinate transformation
    x, y = transformer.transform(longitude_deg, latitude_deg)

    # Subtract the initial coordinates to start from 0
    x -= initial_longitude_deg
    y -= initial_latitude_deg
    x = x * 3.28084
    y = y * 3.28084
    distances = (x - path[:, 0]) ** 2 + (y - path[:, 1]) ** 2
    # Find indices where the condition is satisfied
    ii = np.where(distances < L1 ** 2)[0]

    # Find the maximum value in the array ii
    iii = np.max(ii)
    aim_point = path[iii, :]
    # Define vectors
    v1 = np.array([xd, yd])
    v2 = np.array([aim_point[0] - x, aim_point[1] - y])

    dot_product = np.dot(v1, v2)
    magnitude_v1 = np.linalg.norm(v1)
    magnitude_v2 = np.linalg.norm(v2)

    cosine_theta = dot_product / (magnitude_v1 * magnitude_v2)

    # Calculate the angle in radians
    eta = np.arccos(cosine_theta)
    cross_product = np.cross([v1[0], v1[1], 0], [v2[0], v2[1], 0])
    eta = -eta * np.sign(cross_product[2])
    # Calculate phi_d
    phi_r = np.arctan(2 * (U0 ** 2) * np.sin(eta) / (L1 * 32.17))
    phi_degrees = phi_r * (180 / 3.141592653589793)
    # Clip phi_d to the specified limits
    philim_deg = 30
    print(phi_degrees)
    phi_degrees = np.clip(phi_degrees, -philim_deg, philim_deg)

    pid_aileron_def = pid_roll.compute(phi_degrees, roll_deg)
    pid_elevator_def = -pid_flightpath.compute(target_flightpath_angle,
                                               pitch_deg - alpha_deg)

    data_to_send = str(pid_aileron_def) + "\t" + str(pid_elevator_def) + "\t" + str(0) + "\n"
    send_data(client_2_flightgear, data_to_send)

    print(f" No {xd}, Ea {yd}, U0 {U0}, {str(pid_aileron_def)}")
    print(f" Command {phi_degrees}, x {x}, y {y}")
    print(f" elv {str(pid_elevator_def)}, ail {str(pid_aileron_def)}, eta {np.sin(eta)}")

    ax.scatter(aim_point[0], aim_point[1], label='Aim point', c='yellow', zorder=3)
    ax.plot(path[:, 0], path[:, 1], label='Path')
    ax.scatter(x, y, label='Aircraft location', c='red')

    if previous_x_values and previous_y_values:
        ax.plot(previous_x_values + [x], previous_y_values + [y], linestyle='--', linewidth=1, color='orange',
                label='Followed Path')

        # Update the previous values
    previous_x_values.append(x)
    previous_y_values.append(y)

    v1_normalized = v1 / np.linalg.norm(v1) * L1
    v2_normalized = v2 / np.linalg.norm(v2) * L1
    ax.quiver(x, y, v1_normalized[0], v1_normalized[1], angles='xy', scale_units='xy', scale=1, color='blue',
              label='Speed Vector')
    ax.quiver(x, y, v2_normalized[0], v2_normalized[1], angles='xy', scale_units='xy', scale=1, color='green',
              label='L1 Vector')

    circle = plt.Circle((x, y), L1, color='blue', fill=False)

    ax.add_artist(circle)
    ax.legend()
    ax.grid(True)

    # Update the plot
    plt.pause(0.1)  # Pause for 0.1 seconds to allow the plot to update
    # Clear the previous plot
    ax.clear()

# time.sleep(0.5)
