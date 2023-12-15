The purpose of this project was to achieve a 3D path following algorithm for a fixed-wing aircraft in a three-dimensional FlightGear environment.


The code interacts with FlightGear via a UDP socket connection to send and receive data.


The desired aircraft states are obtained through given signals from previously tuned PID controllers, and the current PID parameters are tuned for a Cessna 172 aircraft. Three-dimensional path following can be broken into two parts: an altitude controller and 2D trajectory tracking in the xy plane.


The path tracking logic is obtained from:

"Park, Sanghyuk, et al. 'A New Nonlinear Guidance Logic for Trajectory Tracking.' AIAA Guidance, Navigation, and Control Conference and Exhibit, 22 June 2004, https://doi.org/10.2514/6.2004-4900"

Here, the roll angle to PID controller is generated to achieve the desired bank angle.


Finally, plots were added to the trajectory, followed path, and live aircraft location in the air for better visualization.


To start the program:

1. add control.xml file to FlightGear 2020.3\data\Protocol folder

2. Open Cessna 172 from Flightgear and in Additional settings section type:
fgfs --generic=socket,out,10,127.0.0.1,5051,udp,control --generic=socket,in,10,127.0.0.1,5052,udp,control --httpd=5405 --disable-sound
(the last two are optional)

3. Choose the path or generate your own in paths.py
