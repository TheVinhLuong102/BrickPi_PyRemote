import socket
import time
import picamera

# Connect a client socket to my_server:8000 (change my_server to the
# hostname of your server)
client_socket = socket.socket()
client_socket.connect(('192.168.179.21', 8000))

# Make a file-like object out of the connection
connection = client_socket.makefile('wb')
try:
    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)
        camera.framerate = 21
        # Start a preview and let the camera warm up for 2 seconds
        camera.start_preview()
        time.sleep(2)
        # Start recording, sending the output to the connection for 60
        # seconds, then stop
        camera.start_recording(connection, format='h264')
        offs=0.0
	for i in range(600):
		
		camera.zoom=(offs,offs,0.5,0.5)
		camera.wait_recording(0.01)
		offs +=0.001
		if offs > 0.5: offs=0.0
        camera.stop_recording()
finally:
	connection.close()


















