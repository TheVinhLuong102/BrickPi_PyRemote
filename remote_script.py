__author__ = 'anton'

import socket
import sdl2
import time
import subprocess
import shlex
# import threading

try:
    import cPickle as pickle
except:
    import pickle
from oculusvr import *

# ##########Constants & configuration################
# Start a video process or not
RCV_VIDEO = True

# Oculus VR configuration
OCULUS_ENABLED = False

# initialise joysticking
sticks = sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK)

# Gamepad config
sixaxis = {
    'gamepad_num': 0,
    'gp_object': sdl2.SDL_JoystickOpen(0),  # sixaxis is the first gamepad on my system
    'stick_range': (-32768,32768),  # min stick position, max stick position. Input range.
    'sticks': {
        'look_h': {'id':2, 'range':(-1000, 1000)},  # stick id and output range. Invert the numbers to invert the axis.
        'look_v': {'id':3, 'range':(200, -200)},
        'move_x': {'id':0, 'range':(-100, 100)},
        'move_y': {'id':1, 'range':(100, -100)}
    },
    'btns': {
        'dpad_up': 4,
        'dpad_right': 5,
        'dpad_down': 6,
        'dpad_left': 7,
        'btn_l2': 8,  # bottom shoulder buttons
        'btn_r2': 9,
        'btn_l1': 10,  # top shoulder buttons
        'btn_r1': 11,
        'btn_A': 14,
        'btn_B': 13,
        'btn_X': 15,
        'btn_Y': 12,
        'btn_start': 3,
        'btn_select': 0
    }
}


# Remote host configuration for opening sockets
HOST = 'brickpiplus'  # The remote RPi with the server script running
PORT = 50007  # The same port as used by the server
MY_IP = '192.168.179.21'

rotationC = 100  # constants to convert input to degrees or sometheing
rollC = 1115 - 1000
translationC = 2000

X_AXIS = 0.0  # rotation of headset
Y_AXIS = 0.0
Z_AXIS = 0.0


# General speed of the program
FRAMERATE = 50  # Number of loops (packets to send) per second






# ############### Helper functions ####################
def scale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.

    val: float or int
    src: tuple
    dst: tuple

    example: print scale(99, (0.0, 99.0), (-1.0, +1.0))
    """
    return (float(val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]


def normalized_stick_value(gp, axis, tgt_range, deadzone=5):
    stick_value = scale(sdl2.SDL_JoystickGetAxis(gp['gp_object'], axis), gp['stick_range'], tgt_range)
    deadzone_range = tuple(n*deadzone/100.0 for n in tgt_range)
    if min(deadzone_range) < stick_value < max(deadzone_range):
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
    """positioning data gathered here"""

    global X_AXIS
    global Y_AXIS
    global Z_AXIS
    global OCULUS_ENABLED

    ss = ovrHmd_GetTrackingState(hmd, ovr_GetTimeInSeconds())
    pose = ss.HeadPose


    # This stuff is for projecting OpenGL images over the video. TODO
    # global xO
    # global yO
    # global xOffset
    # global yOffset
    # xOffset = int(pose.ThePose.Orientation.y * translationC)
    # yOffset = int(pose.ThePose.Orientation.x * translationC)

    X_AXIS = -pose.ThePose.Orientation.x * rotationC
    Y_AXIS = -pose.ThePose.Orientation.y * rotationC
    Z_AXIS = -pose.ThePose.Orientation.z * rollC

    if (X_AXIS, Y_AXIS, Z_AXIS) == (0, 0, 0):
        # print "No oculus data"
        OCULUS_ENABLED = False
    else:
        # print str(X_AXIS) + ", " + str(Y_AXIS) + ", " + str(Z_AXIS)
        OCULUS_ENABLED = True


def get_gamepad_state(gp):
    # update joystick info
    sdl2.SDL_PumpEvents()
    gp_state = {}

    for stick in gp['sticks']:
        gp_state[stick] = normalized_stick_value(gp, gp['sticks'][stick]['id'], gp['sticks'][stick]['range'])

    for btn in gp['btns']:
        gp_state[btn] = sdl2.SDL_JoystickGetButton(gp['gp_object'], gp['btns'][btn])

    if OCULUS_ENABLED:
        gp_state['look_h'] = int(Y_AXIS * 10)  # normalized_stick_value(gp,2,stick_max=300),
        gp_state['look_v'] = int(X_AXIS * -1.9)  # normalized_stick_value(gp,3) * gp['invert_y'],

    return gp_state


# ################## Initialization ###################

# Open socket to the Raspberry Pi
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

# Send our IP address across, and maybe some other config info
# handshake = {'ip_addr': socket.gethostbyname(socket.gethostname())} # the slower way is: socket.gethostbyname(socket.getfqdn())
handshake = {'ip_addr': MY_IP}  # the slower way is: socket.gethostbyname(socket.getfqdn())
msg = pickle.dumps(handshake)
s.send(msg)
time.sleep(3)

# Wait for answer
data = s.recv(1024)
print 'Handshake rcv:', repr(data)

# start oculus rift
if OCULUS_ENABLED:
    ovr_Initialize()
    hmd = ovrHmd_Create(0)
    ovrHmd_ConfigureTracking(hmd,
                             ovrTrackingCap_Orientation |
                             ovrTrackingCap_MagYawCorrection,
                             0
    )

# New throttler
wait = throttler(FRAMERATE)

if RCV_VIDEO:  # Start video player
    cmd = "gst-launch-1.0 udpsrc port=5000 ! application/x-rtp, payload=96, width=1280, height=720 ! rtpjitterbuffer ! rtph264depay ! decodebin ! glimagesink"
    args = shlex.split(cmd)
    vidprocess = subprocess.Popen(args, stdin=subprocess.PIPE)


# ##################Main Loop#######################
while 1:
    try:
        if OCULUS_ENABLED: get_oculus_data()

        gp_data = get_gamepad_state(sixaxis)
        msg = pickle.dumps(gp_data)
        s.send(msg)

        if gp_data['btn_Y']:
            print 'stopping'

            s.close()
            break
        else:
            data = s.recv(1024)  #read back to make sure we can send again. Also nice to get sensor readings.
            print 'Received:', repr(data)

        wait.throttle()
    except KeyboardInterrupt:
        if RCV_VIDEO:
            print "Cleaning up video..."
            vidprocess.terminate()
            break