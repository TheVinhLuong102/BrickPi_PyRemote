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

# As an alternative to the gamepad, you can also configure this script to use
# an oculus rift for input for the camera movement.
#from oculusvr import *





############Constants & configuration################

# Start a gstreamer instance to receive streaming video if true.
RCV_VIDEO = True

# Remote host configuration for opening sockets
HOST = 'brickpiplus'  # The remote RPi with the server script running. Can also be ip number.
PORT = 50007  # The same port as used by the server
MY_IP = '192.168.179.21'


# BrickPi Constants for robot layout config
PORT_A = 0
PORT_B = 1
PORT_C = 2
PORT_D = 3

PORT_1 = 0
PORT_2 = 1
PORT_3 = 2
PORT_4 = 3

# Oculus VR configuration
OCULUS_ENABLED = False
rotationC = 100  # constants to convert input to degrees or sometheing
rollC = 1115 - 1000
translationC = 2000

X_AXIS = 0.0  # rotation of headset
Y_AXIS = 0.0
Z_AXIS = 0.0

# initialise joysticking
sticks = sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK)

# Gamepad config
sixaxis = {
    'gamepad_num': 0,
    'gp_object': sdl2.SDL_JoystickOpen(0),  # sixaxis is the first gamepad on my system
    'stick_range': (-32768, 32768),  # min stick position, max stick position. Input range.
    'sticks': {
        'look_h': {'id': 2, 'invert': 1},
        'look_v': {'id': 3, 'invert': -1},
        'move_x': {'id': 0, 'invert': 1},
        'move_y': {'id': 1, 'invert': 1}
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

# Robot configuration
robot = {
    'motors': {
        'motor_A': {
            'port': PORT_A,
            'control': 'look_h',
            'type': 'servo',
            'range': (150, -150),
            'trim_down': ['dpad_left'],
            'trim_up': ['dpad_right'],
            'trim_step': 5
        },
        'motor_B': {
            'port': PORT_B,
            'control': 'look_v',
            'type': 'servo',
            'range': (-100, 100),
            'co_rotate': 'motor_A', #TODO refactor to 'mix' which is a more common name.
            'co_rotate_pos': -36.0 / 56, #mix_position
            'co_rotate_speed': 0.4, #mix_speed
            'trim_down': ['dpad_up'],
            'trim_up': ['dpad_down'],
            'trim_step': 10
        },
        'motor_C': {  # shoot
                      'port': PORT_C,
                      'control': 'btn_r2',
                      'type': 'servo',
                      'range': (0, -6000),
                      'trim_down': ['dpad_up', 'btn_l2'],
                      'trim_up': ['dpad_down', 'btn_l2'],
                      'trim_step': 10
        },
        'motor_D': {  # forward/rev
                      'port': PORT_D,
                      'control': 'move_y',
                      'type': 'speed',
                      'range': (200, -200)
        }
    },
    'sensors': {}
}




# General speed of the program
FRAMERATE = 50  # Number of loops (packets to send) per second







################# Helper functions ##################
def scale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.

    val: float or int
    src: tuple
    dst: tuple

    example: print scale(99, (0.0, 99.0), (-1.0, +1.0))
    """
    return (float(val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]


def scaled_stick_value(gp, axis, invert, deadzone_pct=4):
    stick_value = scale(sdl2.SDL_JoystickGetAxis(gp['gp_object'], axis),
                        gp['stick_range'],
                        (-32768, 32768)
    ) * invert

    deadzone_range = tuple(n * deadzone_pct / 100.0 for n in (-32768, 32768))

    if min(deadzone_range) < stick_value < max(deadzone_range):
        return 0
    else:
        return int(stick_value)


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
        gp_state[stick] = scaled_stick_value(gp, gp['sticks'][stick]['id'], gp['sticks'][stick]['invert'])

    for btn in gp['btns']:
        gp_state[btn] = sdl2.SDL_JoystickGetButton(gp['gp_object'], gp['btns'][btn])
        #this way a pressed button outputs a number equivalent to a fully bent stick

    if OCULUS_ENABLED:
        gp_state['look_h'] = int(Y_AXIS * 10)  # scaled_stick_value(gp,2,stick_max=300),
        gp_state['look_v'] = int(X_AXIS * -1.9)  # scaled_stick_value(gp,3) * gp['invert_y'],

    return gp_state


#################### Initialization #################

# Open socket to the Raspberry Pi
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

# Send our IP address across, and maybe some other config info
# handshake = {'ip_addr': socket.gethostbyname(socket.gethostname())} 
# the slower way is: socket.gethostbyname(socket.getfqdn())
# Alas the above doesn't work on all networks. Manual ip config is more robust.
handshake = {'ip_addr': MY_IP, 'robot_type': robot}  
msg = pickle.dumps(handshake)
s.send(msg)
time.sleep(3)

# Wait for answer
data = s.recv(1024*2)
print 'Handshake rcv:', pickle.loads(data)

# start oculus rift if needed
if OCULUS_ENABLED:
    ovr_Initialize()
    hmd = ovrHmd_Create(0)
    ovrHmd_ConfigureTracking(hmd,
                             ovrTrackingCap_Orientation |
                             ovrTrackingCap_MagYawCorrection,
                             0
    )

# New throttler #TODO refactor this name to a more legible one.
wait = throttler(FRAMERATE)

# Start video player if needed
if RCV_VIDEO:  
    cmd = "gst-launch-1.0 udpsrc port=5000 ! application/x-rtp, payload=96, width=1280, height=720 ! rtpjitterbuffer ! rtph264depay ! decodebin ! glimagesink"
    args = shlex.split(cmd)
    vidprocess = subprocess.Popen(args, stdin=subprocess.PIPE)





################### Main Loop #######################

while 1:
    try:
        if OCULUS_ENABLED: get_oculus_data()

        gp_data = get_gamepad_state(sixaxis)
        msg = pickle.dumps(gp_data)
        # print gp_data #debug
        s.send(msg)

        data = s.recv(1024*4)  # read back to make sure we can send again. Also nice to get sensor readings.
        rcv = pickle.loads(data)
        print rcv
        wait.throttle()
    except: 
        if RCV_VIDEO:
            print "Cleaning up video..."
            vidprocess.terminate()
            break