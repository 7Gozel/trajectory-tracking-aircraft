import socket
import numpy as np
import matplotlib.pyplot as plt
from pyproj import Proj, Transformer
from PID_autop import PID
from paths import generate_path


path = generate_path(6)
# start FlightGear
# fgfs --generic=socket,out,10,127.0.0.1,5051,udp,control --generic=socket,in,10,127.0.0.1,5052,udp,control --httpd=5405 --disable-sound
# Socket connection
UDP_IP = "127.0.0.1"
UDP_Port_input = 5052
UDP_Port_output = 5051
client_2_flightgear = (UDP_IP, UDP_Port_input)


def send_data(ip_and_port, data):
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data_bytes = data.encode('utf-8')
    send_sock.sendto(data_bytes, ip_and_port)
    send_sock.close()


def receive_data(sock_connection):
    data, address = sock_connection.recvfrom(1024)
    return data.decode('utf-8')


pid_flightpath = PID(0.035, 0.04, 0.001, 0.1, 0.5, -0.5)
pid_roll = PID(0.003, 0.01, 0.002, 0.1, 0.25, -0.25)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.grid(True)

previous_x_values = []
previous_y_values = []
previous_z_values = []
wgs84 = Proj(init='epsg:4326')  # WGS84 coordinate system
initial_latitude_deg = None
initial_longitude_deg = None
first_alt = None
v1_normalized_initial = None
x_initial, y_initial, z_initial = None, None, None

while True:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_Port_output))
    raw_data_output = receive_data(sock)
    data_values = raw_data_output.split()
    roll_deg = float(data_values[0])
    pitch_deg = float(data_values[1])
    alpha_deg = float(data_values[2])
    altitude_ft = float(data_values[4])
    latitude_deg = float(data_values[5])
    longitude_deg = float(data_values[6])

    U0 = float(data_values[10]) * 1.68781  # kt to ft
    xd = float(data_values[8])  # east
    yd = float(data_values[7])  # north
    utm_zone = int((longitude_deg + 180) / 6) + 1  # Calculate the UTM zone
    utm_projection = Proj(proj='utm', zone=utm_zone, ellps='WGS84')
    transformer = Transformer.from_proj(wgs84, utm_projection, always_xy=True)
    if initial_latitude_deg is None or initial_longitude_deg is None:
        # Store the initial coordinates when they are not yet set
        initial_longitude_deg, initial_latitude_deg = transformer.transform(longitude_deg, latitude_deg)
        initial_latitude_deg = initial_latitude_deg

        initial_longitude_deg = initial_longitude_deg
    # Perform the coordinate transformation
    x, y = transformer.transform(longitude_deg, latitude_deg)

    # Subtract the initial coordinates to start from 0
    x -= initial_longitude_deg
    y -= initial_latitude_deg
    x = x * 3.28084  # m to ft
    y = y * 3.28084  # m to ft

    if first_alt is None:
        path[:, 2] = path[:, 2] + altitude_ft
        first_alt = 1

    L1 = 2000
    L2 = 150
    distances_xy = (x - path[:, 0]) ** 2 + (y - path[:, 1]) ** 2
    distance_z = (altitude_ft - path[:, 2]) ** 2
    condition = (distances_xy < L1 ** 2) & (distance_z < L2**2)
    points_within_l = np.where(condition)[0]

    while True:
        condition = (distances_xy < L1 ** 2) & (distance_z < 100 ** 2)
        points_within_l = np.where(condition)[0]
        if len(points_within_l) == 0:
            L1 += 100
            L2 += 5
        else:
            break

    index_of_aim_point = np.max(points_within_l)
    aim_point = path[index_of_aim_point, :]
    v1 = np.array([xd, yd])
    v2 = np.array([aim_point[0] - x, aim_point[1] - y])
    dot_product = np.dot(v1, v2)
    magnitude_v1 = np.linalg.norm(v1)
    magnitude_v2 = np.linalg.norm(v2)
    cosine_theta = dot_product / (magnitude_v1 * magnitude_v2)
    eta = np.arccos(cosine_theta)
    cross_product = np.cross([v1[0], v1[1], 0], [v2[0], v2[1], 0])
    eta = -eta * np.sign(cross_product[2])
    # Calculate phi_d
    phi_radians = np.arctan(2 * (U0 ** 2) * np.sin(eta) / (L1 * 32.17))
    phi_degrees = phi_radians * (180 / 3.141592653589793)
    # Clip phi_d to the specified limits, help when facing opposite the point
    if eta > 3.1416 / 4 or eta < -3.1416 / 4:
        phi_degrees *= 3

    phi_max_allowed = 45
    phi_degrees = np.clip(phi_degrees, -phi_max_allowed, phi_max_allowed)

    target_flightpath_angle = np.arcsin((aim_point[2] - altitude_ft) / U0 / 5)
    target_flightpath_angle *= (180 / 3.141592653589793)
    target_flightpath_angle = max(-5, min(target_flightpath_angle, 5))

    pid_aileron_def = pid_roll.compute(phi_degrees, roll_deg)
    pid_elevator_def = -pid_flightpath.compute(target_flightpath_angle,
                                               pitch_deg - alpha_deg)

    data_to_send = str(pid_aileron_def) + "\t" + str(pid_elevator_def) + "\t" + str(0) + "\n"
    send_data(client_2_flightgear, data_to_send)

    ax.scatter(aim_point[0], aim_point[1], aim_point[2], label='Aim point', c='yellow', zorder=3)
    ax.plot(path[:, 0], path[:, 1], path[:, 2], label='Path')
    ax.scatter(x, y, altitude_ft, label='Aircraft location', c='red')

    if previous_x_values and previous_y_values:
        ax.plot(previous_x_values + [x], previous_y_values + [y], previous_z_values + [altitude_ft], linestyle='--', linewidth=1, color='orange',
                label='Followed Path')

    previous_x_values.append(x)
    previous_y_values.append(y)
    previous_z_values.append(altitude_ft)

    v1_normalized = v1 / np.linalg.norm(v1) * L1
    v2_normalized = v2 / np.linalg.norm(v2) * L1
    if v1_normalized_initial is None:
        v1_normalized_initial = v1_normalized / L1 * 2000

    if x_initial is None and y_initial is None:
        x_initial, y_initial, z_initial = x, y, altitude_ft

    ax.scatter(x_initial, y_initial, z_initial, label='Starting Aircraft location', c='black')
    ax.quiver(x_initial, y_initial, z_initial, v1_normalized_initial[0], v1_normalized_initial[1], 0, color='black', label='Starting Speed Vector(xy)')
    ax.quiver(x, y, altitude_ft, v1_normalized[0], v1_normalized[1], 0, color='blue', label='Speed Vector(xy)')
    ax.quiver(x, y, altitude_ft, v2_normalized[0], v2_normalized[1], 0, color='green', label='L1 Vector(xy)')
    ax.set_xlabel('X (Feet)')
    ax.set_ylabel('Y (Feet)')
    ax.set_zlabel('Altitude (Feet)')
    ax.legend()
    ax.grid(True)
    """ 
    print(f" No {xd}, Ea {yd}, U0 {U0}, {str(pid_aileron_def)}")
    print(f" Command {phi_degrees}, x {x}, y {y}")
    print(f" elv {str(pid_elevator_def)}, ail {str(pid_aileron_def)}, eta {np.sin(eta)}")
    print(f" desired alt {aim_point[2]}, current alt {altitude_ft}, alt error {(aim_point[2] - altitude_ft)}")
      """
    plt.pause(0.25)
    ax.clear()
