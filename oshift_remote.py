__author__ = 'anton'

import socket
import sdl2
import time

try:
    import cPickle as pickle
except:
    import pickle
from oculusvr import *

# ##########Constants & configuration################
# initialise joysticking
sticks = sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK)
LOOK_H_FACTOR = -3      # Amplify control by this number. Do it server-side to avoid loss of resolution

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
HOST = 'brickpiplus'  # The remote host
PORT = 50007  # The same port as used by the server

# Oculus VR configuration

rotationC = 100  # constants to convert input to degrees or sometheing
rollC = 1115 - 1000
translationC = 2000

X_AXIS = 0.0  # rotation of headset
Y_AXIS = 0.0
Z_AXIS = 0.0


# General speed of the program
FRAMERATE = 50  # Number of loops (packets to send) per second



# ############## Helper functions ####################
class throttler(object):
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
    ss = ovrHmd_GetTrackingState(hmd, ovr_GetTimeInSeconds())
    pose = ss.HeadPose

    global X_AXIS
    global Y_AXIS
    global Z_AXIS

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

    #debugging:
    #print str(X_AXIS) + ", " + str(Y_AXIS) + ", " + str(Z_AXIS)


def get_gamepad_state(gp):
    #update joystick info
    sdl2.SDL_PumpEvents()
    #get joystick values in range -100,100
    #it's integer math, so multiplications should go first
    return {'look_h': (sdl2.SDL_JoystickGetAxis(gp['gp_object'], 2) - gp['stick_center']) * 200 * LOOK_H_FACTOR / gp['stick_range'],
                'look_v': (sdl2.SDL_JoystickGetAxis(gp['gp_object'], 3) - gp['stick_center']) * 200 / gp['stick_range'] * gp['invert_y'],
                'move_x': (sdl2.SDL_JoystickGetAxis(gp['gp_object'], 0) - gp['stick_center']) * 200 / gp['stick_range'],
                'move_y': (sdl2.SDL_JoystickGetAxis(gp['gp_object'], 1) - gp['stick_center']) * 200 / gp['stick_range'] * gp['invert_y'],
                'btn_Y': sdl2.SDL_JoystickGetButton(gp['gp_object'], gp['btn_Y']),
                'btn_A': sdl2.SDL_JoystickGetButton(gp['gp_object'], gp['btn_A']),
                'btn_lshoulder': sdl2.SDL_JoystickGetButton(gp['gp_object'], gp['btn_lshoulder']),
                'btn_rshoulder': sdl2.SDL_JoystickGetButton(gp['gp_object'], gp['btn_rshoulder'])
            }

################### Intialisation ###################

# open socket to the Raspberry Pi
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

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
while 1:
    get_oculus_data()
    gp_data = get_gamepad_state(sixaxis)
    msg = pickle.dumps(gp_data)
    s.send(msg)
    print gp_data['btn_A'], gp_data['look_h'], gp_data['look_v']
    if gp_data['btn_Y']:
        print 'stopping'

        s.close()
        break
    else:
        data = s.recv(1024)
        print 'Received:', repr(data)

    wait.throttle()