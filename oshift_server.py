__author__ = 'anton'


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
    def __init__(self,port,KP=3.0,KI=1.5,KD=0.3):
        self.port = port
        self.KP = KP
        self.KI = KI
        self.KD = KD

    def setZero(self,zero_pos):
        self.zero = zero_pos

    def get_power(self,error):
        return error *2

# previous_error = 0
# integral = 0
# start:
#   error = setpoint - measured_value
#   integral = integral + error*dt
#   derivative = (error - previous_error)/dt
#   output = Kp*error + Ki*integral + Kd*derivative
#   previous_error = error
#   wait(dt)
#   goto start


############# Initialize ##########################
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
gp_state = {'look_h': 0, 'look_v': 0, 'move_x': 0, 'move_y': 0, 'btn_start': 0, 'btn_A': 0}
wait = throttler(MOTOR_CMD_RATE)


############ Main threads ######################

class motorControl (threading.Thread):		#This thread is used for keeping the motor running while the main thread waits for user input
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter

    def run(self):

        # Ask BrickPi to update values for sensors/motors
        # Make sure we have something before we start running
        # So we wait until no_values goes 0, which means values updated OK
        no_values = 1
        while no_values:
            no_values = BrickPiUpdateValues()

        # Now we can start!
        position0_A = int(BrickPi.Encoder[PORT_A])
        #position0_B = int(BrickPi.Encoder[PORT_B])


        while running:

            err_A = (BrickPi.Encoder[PORT_A]-position0_A)-gp_state['look_h']
            #err_B = (BrickPi.Encoder[PORT_A]-position0_A)-gp_state['look_v']

            BrickPi.MotorSpeed[PORT_A] = -err_A*2		# Set Speed=0 which means stop
            BrickPiUpdateValues()       # Ask BrickPi to update values for sensors/motors
            wait.throttle()            # Don't overload the brickpi too much

thread1 = motorControl(1, "Thread-1", 1)		#Setup and start the thread
thread1.setDaemon(True)
thread1.start()

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
                        wait.throttle()
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
        running = False				#Stop theread1
        print "Bye"
        break					#Exit