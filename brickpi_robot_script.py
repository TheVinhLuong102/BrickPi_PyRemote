__author__ = 'anton'

# For passing arguments to the script from the command line
import sys

# Communications to the NXT brick omnibot
from nxt.brick import Brick
from nxt.locator import find_one_brick
from nxt.motor import Motor, PORT_A, PORT_B, PORT_C
import nxt.bluesock
import math

# To start the camera subprocess
import shlex
import picamera

# To open sockets en receive data from client
import socket, time, select

try:
    import cPickle as pickle
except:
    import pickle

# To run motors on the brickpi, in a separate thread
from BrickPi import *  # import BrickPi.py file to use BrickPi operations
import threading
import subprocess



# ############# Constants & settings ################

if 'bt' in sys.argv:
    BLUETOOTH = True
else:
    BLUETOOTH = False

if 'vid' in sys.argv:
    VIDEO = True
else:
    VIDEO = False

BRICK_ADDR = '00:16:53:0E:1C:AC'  # change this to the bt address of your brick
BRICK_NAME = 'NXT'  # fallback if the address is not found

CONNECTION_LIST = []  # list of socket clients
RECV_BUFFER = 4096  # Advisable to keep it as an exponent of 2
PORT = 50007  # data port
VIDEO_PORT = 5000

MOTOR_CMD_RATE = 30  # Max number of motor commands per second
BT_CMD_RATE = 30

MOTOR_A_RANGE = (-300, 300) #for safety TODO
MOTOR_B_RANGE = (-100, 100)
MOTOR_C_RANGE = (0, 400)

RASPIVID_CMD = "raspivid -t 999999 -b 2000000 -o -"
STREAM_CMD = "gst-launch-1.0 -e -vvv fdsrc ! h264parse ! rtph264pay pt=96 config-interval=5 ! udpsink host={0} port={1}"

FRAME_RATE = 24
VIDEO_W = 1280
VIDEO_H = 720
BITRATE = 1000000





########### Helper functions ######################
def clean_up():
    global running, video_playing
    running = False  #Stop threads
    video_playing = False
    for sock in CONNECTION_LIST:
        sock.close()
    print "Bye"


def clamp(n, (minn, maxn)):
    return max(min(maxn, n), minn)


class Throttler(object):
    def __init__(self, framerate):
        self.fps = framerate
        self.timestamp = time.time()

    def throttle(self):
        wait_time = 1.0 / self.fps - (
            time.time() - self.timestamp)  #has enough time passed? If not, this is the remainder
        if wait_time > 0:
            time.sleep(wait_time)
        self.timestamp = time.time()


class motorPID(object):
    def __init__(self, KP=.6, KI=0.05, KD=0.0):
        self.Kp = KP
        self.Ki = KI
        self.Kd = KD
        self.integral = 0
        self.prev_error = 0
        self.timestamp = time.time()
        self.zero = 0

    def set_zero(self, zero_pos):
        self.zero = zero_pos

    def inc_zero(self, increment):
        self.zero += increment

    def get_power(self, error):
        dt = time.time() - self.timestamp
        error -= self.zero
        self.integral += error * dt  #shouldn't the integral be emptied sometime?
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        self.timestamp = time.time()
        return int(-output)


############# Initialize ##########################

#  Setup BrickPi and motors
print "Revving up engines"
BrickPiSetup()  # setup the serial port for communication
BrickPi.MotorEnable[PORT_A] = 1  #Enable the Motor A, panning horizontal
BrickPi.MotorEnable[PORT_B] = 1  #Enable the Motor B, panning vertical
BrickPi.MotorEnable[PORT_C] = 1  #Enable the Motor C, cannon

#BrickPi.SensorType[PORT_4] = TYPE_SENSOR_ULTRASONIC_CONT	#Setting the type of sensor at PORT4
#BrickPiSetupSensors()  #Send the properties of sensors to BrickPi

# Initialize server socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(("0.0.0.0", PORT))
server_socket.listen(10)

# Add server socket to the list of readable connections
CONNECTION_LIST.append(server_socket)
print "Chat server started on port " + str(PORT)

# Initialize globals
video_playing = False
running = True
gp_state = {'look_h': 0, 'look_v': 0, 'move_x': 0, 'move_y': 0, 'btn_start': 0, 'btn_A': 0, 'btn_Y': 0,
            'btn_l1': 0,
            'btn_r1': 0,
            'btn_r2': 0,
            'btn_r2': 0,
            'dpad_up': 0,
            'dpad_down': 0,
            'dpad_left': 0,
            'dpad_right': 0
            }

motorloop = Throttler(MOTOR_CMD_RATE)
btloop = Throttler(BT_CMD_RATE)








############ Main threads ######################

class sendVideo(threading.Thread):
    def __init__(self, ip_addr):
        threading.Thread.__init__(self)
        self.ip_addr = ip_addr


    def run(self):
        cmd = shlex.split(STREAM_CMD.format(self.ip_addr, VIDEO_PORT))
        streamer = subprocess.Popen(cmd, stdin=subprocess.PIPE)

        try:
            with picamera.PiCamera() as camera:
                camera.resolution = (VIDEO_W, VIDEO_H)
                camera.framerate = FRAME_RATE

                # Start a preview and let the camera warm up for 2 seconds
                camera.start_preview()
                time.sleep(2)
                camera.start_recording(streamer.stdin, format='h264', bitrate=BITRATE)
                while video_playing:
                    camera.wait_recording(1)
                camera.stop_recording()
        finally:
            streamer.terminate()


class btRemoteControl(threading.Thread):
    def run(self):
        # Start a BT connection over to the NXT
        print "Initializing Bluetooth"
        try:
            brick = nxt.bluesock.BlueSock(BRICK_ADDR).connect()
        except:
            print "Brick {0} not found. Trying search by name...".format(BRICK_ADDR)
            try:
                brick = find_one_brick(name=BRICK_NAME)
            except:
                print "Brick {0} not found".format(BRICK_NAME)
            return

        bt_motors = (Motor(brick, PORT_A), Motor(brick, PORT_B), Motor(brick, PORT_C))
        while running:
            # use shoulder buttons on the game pad to make the omnibot rotate around it's axis.
            turnpower = 0
            if gp_state['btn_r1']: turnpower = 60
            if gp_state['btn_l1']: turnpower = -60

            # convert joystick x and y to a direction and power (deviation from the centre)
            joy_direction = math.atan2(gp_state['move_x'], gp_state['move_y'])  # in radians
            joy_power = (gp_state['move_x'] ** 2 + gp_state['move_y'] ** 2) ** 0.5  # pythagoras

            i = 0
            for motor in bt_motors:
                # for each motor the angle has a different offset (0, 120 and 240 degrees)
                angle = i * 2 * 3.1415 / 3 + joy_direction

                # motor power calculation. A simple sin.
                motorpower = math.sin(angle) * joy_power + turnpower
                motorpower = round(clamp(motorpower, (-100, 100)))
                motor.run(motorpower, regulated=True)
                i += 1

            # wait a bit before sending more commands. If we don't the BT buffer overflows.
            btloop.throttle()


class motorControl(
    threading.Thread):  #This thread is used for keeping the motor running while the main thread waits for user input

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

        pid_control_c = motorPID()
        pid_control_c.set_zero(int(BrickPi.Encoder[PORT_C]))

        while running:
            if len(CONNECTION_LIST) > 1:  #only turn motors if there's a client connected on the socket
                #Zero point calibration with dpad
                if gp_state['dpad_up']:
                    if gp_state['btn_l2']:
                        pid_control_c.inc_zero(50)
                    else:
                        pid_control_b.inc_zero(5)
                    time.sleep(0.3)
                if gp_state['dpad_down']:
                    if gp_state['btn_l2']:
                        pid_control_c.inc_zero(-50)
                    else:
                        pid_control_b.inc_zero(-5)
                    time.sleep(0.3)
                if gp_state['dpad_left']:
                    pid_control_a.inc_zero(5)
                    time.sleep(0.3)
                if gp_state['dpad_right']:
                    pid_control_a.inc_zero(-5)
                    time.sleep(0.3)


                err_A = (BrickPi.Encoder[PORT_A] - gp_state['look_h'])

                err_B = (BrickPi.Encoder[PORT_B] + gp_state['look_v'])

                # when the head turns horizontally, the vertical axis has turn to match
                # the rotation, because the axles are concentric.
                # the horizontal rotation is in a ratio of 56:8, using a large turntable and an 8 tooth gear

                v_look_zero_offset = BrickPi.Encoder[PORT_A] - pid_control_a.zero  # get rotation of motor A
                err_B -= v_look_zero_offset * 8.0 / 56 - err_A / 10.0  # offset motor B target with this number

                if gp_state['btn_r2']: #shoot with the bottom right shoulder button
                    err_C = (BrickPi.Encoder[PORT_C] + 6000)
                else:
                    err_C = (BrickPi.Encoder[PORT_C] - 0)

                BrickPi.MotorSpeed[PORT_A] = pid_control_a.get_power(err_A)
                BrickPi.MotorSpeed[PORT_B] = pid_control_b.get_power(err_B)
                BrickPi.MotorSpeed[PORT_C] = pid_control_c.get_power(err_C)

                BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors


            else:  #No client connected on the socket. Power down the motors.
                BrickPi.MotorSpeed[PORT_A] = 0
                BrickPi.MotorSpeed[PORT_B] = 0
                BrickPi.MotorSpeed[PORT_C] = 0
            motorloop.throttle()  # Don't overload the brickpi too much, wait a bit.


thread1 = motorControl()  #Setup and start the thread
thread1.setDaemon(True)
thread1.start()

if BLUETOOTH:  #Start BT thread
    thread2 = btRemoteControl()
    thread2.setDaemon(True)
    thread2.start()

while True:  #Main loop
    try:
        # Get the list sockets which are ready to be read through select
        read_sockets, write_sockets, error_sockets = select.select(CONNECTION_LIST, [], [])
        for sock in read_sockets:

            #New connection
            if sock == server_socket:
                # Handle the case in which there is a new connection recieved through server_socket
                sockfd, addr = server_socket.accept()
                CONNECTION_LIST.append(sockfd)
                print "Client (%s, %s) connected" % addr


            #Some incoming message from a connected client
            else:
                # Data recieved from client, process it
                try:
                    #In Windows, sometimes when a TCP program closes abruptly,
                    # a "Connection reset by peer" exception will be thrown
                    data = sock.recv(RECV_BUFFER)
                    rcvd_dict = pickle.loads(data)
                    sock.send('OK ... ')

                    if 'ip_addr' in rcvd_dict:
                        if VIDEO:
                            video_playing = True
                            thread3 = sendVideo(rcvd_dict['ip_addr'])  #Setup and start the thread
                            thread3.setDaemon(True)
                            thread3.start()
                            print "Started video"

                    else:
                        gp_state = rcvd_dict

                        # acknowledge


                # client disconnected, so remove from socket list
                except:
                    #broadcast_data(sock, "Client (%s, %s) is offline" % addr)
                    print "Client (%s, %s) is offline" % addr
                    sock.close()
                    CONNECTION_LIST.remove(sock)
                    clean_up()

                    #continue
                    break
    except KeyboardInterrupt:  #Triggered by pressing Ctrl+C. Time to clean up.
        clean_up()
        break  #Exit