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

# Gamepad config
sixaxis = {
    'gamepad_num': 0,
    'gp_object': sdl2.SDL_JoystickOpen(0),  # sixaxis
    'stick_range': 65536,  # max stick position - min stick position
    'stick_center': 0,
    'left_shoulder_btn': 8,
    'right_shoulder_btn': 9,
    'invert_y': -1,
    'abort_btn': 12
}

# Remote host configuration for opening sockets
HOST = 'brickpi'  # The remote host
PORT = 50007  # The same port as used by the server

# Oculus VR configuration

rotationC = 100  #constants to convert input to degrees or sometheing
rollC = 1115 - 1000
translationC = 2000

X_AXIS = 0.0  #rotation of headset
Y_AXIS = 0.0
Z_AXIS = 0.0


###################Intialisation###################

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


###############Helper functions####################

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

    print str(X_AXIS) + ", " + str(Y_AXIS) + ", " + str(Z_AXIS)


def get_gamepad_state(gp):
    #update joystick info
    sdl2.SDL_PumpEvents()
    #joy_x is an int in range -100,100
    gp_state = {'look_h': (sdl2.SDL_JoystickGetAxis(gp['gp_object'], 2) - gp['stick_center']) * 200 / gp['stick_range'],
                'look_v': (sdl2.SDL_JoystickGetAxis(gp['gp_object'], 3) - gp['stick_center']) * 200 / gp['stick_range']

    }

    return gp_state['look_h']

###################Main Loop#######################
while 1:
    get_oculus_data()
    gp_data = get_gamepad_state(sixaxis)
    msg = pickle.dumps(gp_data)
    s.send(msg)

    if sdl2.SDL_JoystickGetButton(sixaxis['gamepad'], sixaxis['abort_btn']):
        print 'stopping'

        s.close()
        break
    else:
        data = s.recv(1024)
        print 'Received:', repr(data)