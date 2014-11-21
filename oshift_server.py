__author__ = 'anton'


import socket, time, select
try:
    import cPickle as pickle
except:
    import pickle
from BrickPi import *   #import BrickPi.py file to use BrickPi operations
import threading

#Setup socket server
#Socket listening stuff
CONNECTION_LIST = []    # list of socket clients
RECV_BUFFER = 4096 # Advisable to keep it as an exponent of 2
PORT = 50007

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# this has no effect, why ?
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(("0.0.0.0", PORT))
server_socket.listen(10)

# Add server socket to the list of readable connections
CONNECTION_LIST.append(server_socket)

print "Chat server started on port " + str(PORT)

BrickPiSetup()  # setup the serial port for communication

BrickPi.MotorEnable[PORT_A] = 1 #Enable the Motor A
#BrickPi.MotorEnable[PORT_D] = 1 #Enable the Motor D

#BrickPi.SensorType[PORT_4] = TYPE_SENSOR_ULTRASONIC_CONT	#Setting the type of sensor at PORT4

BrickPiSetupSensors()   #Send the properties of sensors to BrickPi

#setup globals
running = True
joy_x = 0

class motorControl (threading.Thread):		#This thread is used for keeping the motor running while the main thread waits for user input
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter

    def run(self):
        global joy_x
        result = 1
        while result:
            result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors
            #print result
        position0 = int(BrickPi.Encoder[PORT_A])
        while running:

            Perr = (BrickPi.Encoder[PORT_A]-position0)-joy_x
            print Perr
            BrickPi.MotorSpeed[PORT_A] = -Perr*2		# Set Speed=0 which means stop
            BrickPiUpdateValues()       # Ask BrickPi to update values for sensors/motors
            time.sleep(.015)              # sleep for 200 ms

thread1 = motorControl(1, "Thread-1", 1)		#Setup and start the thread
thread1.setDaemon(True)
thread1.start()

while True:
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
                        joy_x = pickle.loads(data)

                        # echo back the client message
                        if data:
                            sock.send('OK ... ' + data)

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