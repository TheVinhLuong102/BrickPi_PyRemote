import socket
import time
import picamera
import subprocess
import math

cmdline = ['gst-launch-1.0','-e','-vvv','fdsrc','!','h264parse','!','rtph264pay','pt=96','config-interval=5','!','udpsink','host=192.168.179.21','port=5000']
streamer = subprocess.Popen(cmdline, stdin=subprocess.PIPE)

try:
    with picamera.PiCamera() as camera:
        camera.resolution = (1280,768)
        camera.framerate = 24
        # Start a preview and let the camera warm up for 2 seconds
        camera.start_preview()
        time.sleep(2)
        # Start recording, sending the output to the connection for 60
        # seconds, then stop
        camera.start_recording(streamer.stdin, format='h264')
        for i in range(60000):
#            camera.zoom = (math.sin((i%1000)/1000.0*2*math.pi)**2/.8,.5,.2,.2)
            camera.wait_recording(0.01)
        camera.stop_recording()
finally:
    streamer.terminate()

