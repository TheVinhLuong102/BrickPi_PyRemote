__author__ = 'anton'

import socket
import sdl2
import time
import subprocess
import shlex
import threading

try:
    import cPickle as pickle
except:
    import pickle
from oculusvr import *

# ##########Constants & configuration################
# Start a video process or not
START_VIDEO = False

# initialise joysticking
sticks = sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK)


# Gamepad config
sixaxis = {
    'gamepad_num': 0,
    'gp_object': sdl2.SDL_JoystickOpen(0),  # sixaxis is the first gamepad on my system
    'stick_range': 65536,  # max stick position - min stick position
    'stick_center': 0,
    'btn_lshoulder': 8,
    'btn_rshoulder': 9,
    'btn_A': 14,
    'btn_B': 13,
    'btn_X': 15,
    'btn_Y': 12,
    'invert_y': -1, #-1 if it needs to be inverted, 1 otherwise
    'btn_start': 3,
    'btn_select': 0
}


# Remote host configuration for opening sockets
HOST = 'brickpiplus'  # The remote RPi with the server script running
PORT = 50007  # The same port as used by the server
MY_IP = '192.168.179.21'

# Oculus VR configuration
OCULUS_ENABLED = False

rotationC = 100  # constants to convert input to degrees or sometheing
rollC = 1115 - 1000
translationC = 2000

X_AXIS = 0.0  # rotation of headset
Y_AXIS = 0.0
Z_AXIS = 0.0


# General speed of the program
FRAMERATE = 50  # Number of loops (packets to send) per second



# ############## Helper functions ####################
def normalized_stick_value(gp, axis, stick_max=100, deadzone=5):
    stick_value = (sdl2.SDL_JoystickGetAxis(gp['gp_object'], axis) - gp['stick_center']) #center value arount 0
    stick_value = stick_value * 2 * stick_max / gp['stick_range'] #normalize
    if -deadzone < stick_value < deadzone:
        return 0
    else:
        return stick_value

class throttler(object):
    """
    This class makes sure a loop doesn't run faster than
    it's designated loops per second by calling the throttle method
    at the end of each loop.
    """
    def __init__(self, framerate):
        self.fps = framerate
        self.timestamp = time.time()
    def throttle(self):
        wait_time = 1.0 / self.fps - (time.time() - self.timestamp)
        if wait_time > 0:
            time.sleep(wait_time)
        self.timestamp = time.time()


def get_oculus_data():
    """possitioning data gathered here"""

    global X_AXIS
    global Y_AXIS
    global Z_AXIS
    global OCULUS_ENABLED


    ss = ovrHmd_GetTrackingState(hmd, ovr_GetTimeInSeconds())
    pose = ss.HeadPose


    #This stuff is for projecting OpenGL images over the video. TODO
    # global xO
    # global yO
    # global xOffset
    # global yOffset
    # xOffset = int(pose.ThePose.Orientation.y * translationC)
    # yOffset = int(pose.ThePose.Orientation.x * translationC)

    X_AXIS = -pose.ThePose.Orientation.x * rotationC
    Y_AXIS = -pose.ThePose.Orientation.y * rotationC
    Z_AXIS = -pose.ThePose.Orientation.z * rollC

    if (X_AXIS,Y_AXIS,Z_AXIS) == (0,0,0):
        #print "No oculus data"
        OCULUS_ENABLED = False
    else:
        #print str(X_AXIS) + ", " + str(Y_AXIS) + ", " + str(Z_AXIS)
        OCULUS_ENABLED = True


def get_gamepad_state(gp):
    #update joystick info
    sdl2.SDL_PumpEvents()
    gp_state =  {'look_h': normalized_stick_value(gp,2,stick_max=1800),
                'look_v': normalized_stick_value(gp,3,stick_max=200) * gp['invert_y'],
                'move_x': normalized_stick_value(gp,0),
                'move_y': normalized_stick_value(gp,1) * gp['invert_y'],
                'btn_Y': sdl2.SDL_JoystickGetButton(gp['gp_object'], gp['btn_Y']),
                'btn_A': sdl2.SDL_JoystickGetButton(gp['gp_object'], gp['btn_A']),
                'btn_lshoulder': sdl2.SDL_JoystickGetButton(gp['gp_object'], gp['btn_lshoulder']),
                'btn_rshoulder': sdl2.SDL_JoystickGetButton(gp['gp_object'], gp['btn_rshoulder'])
            }

    if OCULUS_ENABLED:
        gp_state['look_h'] = int(Y_AXIS*10) #normalized_stick_value(gp,2,stick_max=300),
        gp_state['look_v'] = int(X_AXIS*-1.9) #normalized_stick_value(gp,3) * gp['invert_y'],

    return gp_state

################### Intialisation ###################

# open socket to the Raspberry Pi
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

# send our IP address across
#handshake = {'ip_addr': socket.gethostbyname(socket.gethostname())} # the slower way is: socket.gethostbyname(socket.getfqdn())
handshake = {'ip_addr': MY_IP} # the slower way is: socket.gethostbyname(socket.getfqdn())
msg = pickle.dumps(handshake)
s.send(msg)

time.sleep(3)

# wait for answer
data = s.recv(1024)
print 'Handshake rcv:', repr(data)

# start oculus rift
ovr_Initialize()
hmd = ovrHmd_Create(0)
ovrHmd_ConfigureTracking(hmd,
                         ovrTrackingCap_Orientation |
                         ovrTrackingCap_MagYawCorrection,
                         0
)

# new throttler
wait = throttler(FRAMERATE)


###################Main Loop#######################
class VideoPlayer(threading.Thread):
    global vidprocess
    # start receiving video
    if START_VIDEO:
        #cmd = "gst-launch-1.0 -e udpsrc port=5000 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! fpsdisplaysink sync=false text-overlay=false"
        cmd = "gst-launch-1.0 udpsrc port=5000 ! application/x-rtp, payload=96, width=960, height=1080 ! rtpjitterbuffer ! rtph264depay ! decodebin ! glimagesink sync=false"
        args = shlex.split(cmd)
        vidprocess = subprocess.call(args)

thread1 = VideoPlayer()  #Setup and start the thread
thread1.setDaemon(True)
thread1.start()

while 1:
    try:
        get_oculus_data()
        gp_data = get_gamepad_state(sixaxis)
        msg = pickle.dumps(gp_data)
        s.send(msg)
        #print gp_data['btn_A'], gp_data['look_h'], gp_data['look_v']
        if gp_data['btn_Y']:
            print 'stopping'

            s.close()
            break
        else:
            data = s.recv(1024) #read back to make sure we can send again. Also nice to get sensor readings.
            print 'Received:', repr(data)

        wait.throttle()
    except KeyboardInterrupt:
        if START_VIDEO:
            print "Cleaning up video..."
            vidprocess.terminate()
            break