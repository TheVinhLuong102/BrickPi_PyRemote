#!/bin/bash

RPI="brickpiplus"
SERVER_SCRIPT="brickpi_robot_script.py"

#copy the server script over. overwrite switch?
scp $SERVER_SCRIPT pi@$RPI:~/

# Run the server script
# The < and > magic makes sure it runs in the background so ssh command returns

ssh pi@$RPI "python $SERVER_SCRIPT < /dev/null > /tmp/oshiftlogfile 2>&1 &"
sleep 3
echo "Remote server started"
