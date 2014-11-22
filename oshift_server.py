__author__ = 'anton'

from jaraco.nxt import *
from jaraco.nxt.messages import *
import math

import socket, time, select
try:
    import cPickle as pickle
except:
    import pickle
from BrickPi import *   #import BrickPi.py file to use BrickPi operations
import threading


############# Constants & settings ################

#Setup socket server
CONNECTION_LIST = []    # list of socket clients
RECV_BUFFER = 4096      # Advisable to keep it as an exponent of 2
PORT = 50007
MOTOR_CMD_RATE = 30     # Max number of motor commands per second


########### Helper functions ######################
def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

class throttler(object):
    def __init__(self, framerate):
        self.fps = framerate
        self.timestamp = time.time()
    def throttle(self):
        wait_time = 1.0 / self.fps - (time.time() - self.timestamp)
        if wait_time > 0:
            time.sleep(wait_time)
        self.timestamp = time.time()

class motorPID(object):
    def __init__(self,KP=3.0,KI=1.5,KD=0.3):

        self.Kp = KP
        self.Ki = KI
        self.Kd = KD
        self.integral = 0
        self.prev_error = 0
        self.timestamp = time.time()
        self.zero = 0

    def set_zero(self,zero_pos):
        self.zero = zero_pos

    def get_power(self,error):
        dt = time.time()-self.timestamp
        error -= self.zero
        self.integral += error*dt #shouldn't the integral be emptied sometime?
        derivative = (error - self.prev_error)/dt
        output = self.Kp*error + self.Ki*self.integral + self.Kd*derivative
        self.prev_error = error
        self.timestamp = time.time()
        return int(-output)


############# Initialize ##########################

# Start a BT connection over to the NXT
# open the connection. Get this string from preferences>bluetooth>NXT>edit serial ports
conn = Connection('/dev/rfcomm0')  # antons NXT

# Initialize server socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# this has no effect, why ?
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(("0.0.0.0", PORT))
server_socket.listen(10)

# Add server socket to the list of readable connections
CONNECTION_LIST.append(server_socket)
print "Chat server started on port " + str(PORT)

# Setup BrickPi and motors
BrickPiSetup()  # setup the serial port for communication
BrickPi.MotorEnable[PORT_A] = 1 #Enable the Motor A
BrickPi.MotorEnable[PORT_B] = 1 #Enable the Motor B
#BrickPi.SensorType[PORT_4] = TYPE_SENSOR_ULTRASONIC_CONT	#Setting the type of sensor at PORT4
BrickPiSetupSensors()   #Send the properties of sensors to BrickPi

#Initialize globals
running = True
gp_state = {'look_h': 0, 'look_v': 0, 'move_x': 0, 'move_y': 0, 'btn_start': 0, 'btn_A': 0, 'btn_lshoulder': 0, 'btn_rshoulder':0}
wait = throttler(MOTOR_CMD_RATE)


############ Main threads ######################
class btRemoteControl(threading.Thread):
    def run(self):
        while running:
            # use shoulder buttons on the game pad to make the omnibot rotate around it's axis.
            turnpower = 0
            if gp_state['btn_rshoulder']:turnpower = 75
            if gp_state['btn_lshoulder']:turnpower = -75

            # convert joystick x and y to a direction and power (deviation from the centre)
            joy_direction = math.atan2(gp_state['move_y'], gp_state['move_x'])  # in radians
            joy_power = (gp_state['move_x'] ** 2 + gp_state['move_y'] ** 2) ** 0.5  # pythagoras

            # building a list of three motor commands to send over
            cmds = []
            for i in range(3):
                # for each motor the angle has a different offset (0, 120 and 240 degrees)
                angle = i * 2 * 3.1415 / 3 + joy_direction

                # motor power calculation. A simple sin.
                motorpower = math.sin(angle) * joy_power + turnpower

                motorpower = round(clamp(motorpower, -100, 100))

                cmds.append(SetOutputState(
                                          i,
                                          motor_on=True,
                                          set_power=motorpower,
                                          run_state=RunState.running,
                                          use_regulation=True,
                                          regulation_mode=RegulationMode.motor_speed
                                          )
                    )

            map(conn.send, cmds)

            # wait a bit before sending more commands. If we don't the BT buffer overflows.
            time.sleep(0.05)



class motorControl (threading.Thread):		#This thread is used for keeping the motor running while the main thread waits for user input
    def run(self):

        # Ask BrickPi to update values for sensors/motors
        # Make sure we have something before we start running
        # So we wait until no_values goes 0, which means values updated OK
        no_values = 1
        while no_values:
            no_values = BrickPiUpdateValues()

        # Now we can start!
        pid_control_a = motorPID()
        pid_control_a.set_zero(int(BrickPi.Encoder[PORT_A]))

        pid_control_b = motorPID()
        pid_control_b.set_zero(int(BrickPi.Encoder[PORT_B]))
        #position0_A = int(BrickPi.Encoder[PORT_A])
        #position0_B = int(BrickPi.Encoder[PORT_B])


        while running:

            err_A = (BrickPi.Encoder[PORT_A]-gp_state['look_h'])
            err_B = (BrickPi.Encoder[PORT_B]-gp_state['look_v'])

            BrickPi.MotorSpeed[PORT_A] = pid_control_a.get_power(err_A)		# Set Speed=0 which means stop
            BrickPi.MotorSpeed[PORT_B] = pid_control_b.get_power(err_B)
            BrickPiUpdateValues()       # Ask BrickPi to update values for sensors/motors
            wait.throttle()            # Don't overload the brickpi too much

thread1 = motorControl()		#Setup and start the thread
thread1.setDaemon(True)
thread1.start()

thread2 = btRemoteControl()		#Setup and start the thread
thread2.setDaemon(True)
thread2.start()


while True: #Main loop
    try:
        # Get the list sockets which are ready to be read through select
            read_sockets,write_sockets,error_sockets = select.select(CONNECTION_LIST,[],[])

            for sock in read_sockets:

                #New connection
                if sock == server_socket:
                    # Handle the case in which there is a new connection recieved through server_socket
                    sockfd, addr = server_socket.accept()
                    CONNECTION_LIST.append(sockfd)
                    print "Client (%s, %s) connected" % addr

                #Some incoming message from a client
                else:
                    # Data recieved from client, process it
                    try:
                        #In Windows, sometimes when a TCP program closes abruptly,
                        # a "Connection reset by peer" exception will be thrown
                        data = sock.recv(RECV_BUFFER)
                        gp_state = pickle.loads(data)

                        # acknowledge
                        sock.send('OK ... ')

                    # client disconnected, so remove from socket list
                    except:
                        #broadcast_data(sock, "Client (%s, %s) is offline" % addr)
                        print "Client (%s, %s) is offline" % addr
                        sock.close()
                        CONNECTION_LIST.remove(sock)
                        continue
    except KeyboardInterrupt:			#Triggered by pressing Ctrl+C
        running = False				    #Stop threads
        conn.close()                    #close BT rfcomm
        server_socket.close()
        print "Bye"
        break					#Exit