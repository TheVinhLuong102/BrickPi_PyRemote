# About #

This repo contains server and client scripts to remote control a Lego Mindstorms robot. 

Hardware used:
* Macbook
* Oculus Rift DK2
* Dexter Industries BrickPi
* Raspberry Pi model B (1x)
* Raspberry Pi model B+ (1x)
* PiCam (2x)
* WiPi wifi-stick (2x)
* Mobile phone wide angle clip-on lenses (2x)
* Mindstorms NXT 2 set
* Bluetooth dongle
* PS3 sixaxis gamepad (any gamepad should do)
* Large EV3 motors (2x)
* Medium EV3 motor

![Concept drawing](/photos/concept-sm.jpeg?raw=true "Concept drawing")

# Usage #
Get the necessary dependencies, put the server script on the BrickPi and run the remote script on your mac.


# Installation #

## Raspberry Pi ##
Make sure you start with a fresh [Raspbian image](http://www.raspberrypi.org/downloads/). I used Debian Wheezy. 
Don't use the image from dexter industries. BT doesn't 
  work on it and it has meager support for WiFi dongles. On a Mac you can burn the downloaded image with the 'restore 
  backup function in [ApplePi-Baker](http://www.tweaking4all.com/hardware/raspberry-pi/macosx-apple-pi-baker/).
  This should also be possible via the command line, but I didn't bother.

Make sure you set up your image after the first boot.
```bash
sudo raspi-config
```
And set up

1. The PiCamera
2. A custom hostname e.g. 'BrickPi'
3. Whatever else you feel like

If you feel like it, upgrade all of the installed stuff and remove redundant ones after the upgrade
 but it shouldn't be necessary on a new image:
```bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get autoremove
```

Now is also a good time to set up your wifi. It's easy if you have the Pi connected to a display, or if you have
VNC set up. Otherwise you can [set up wifi via the command line](http://www.howtogeek.com/167425/how-to-setup-wi-fi-on-your-raspberry-pi-via-the-command-line/)

Next you'll have to install the BrickPi stuff. Go to http://www.dexterindustries.com/BrickPi/getting-started/pi-prep/ and follow the instructions carefully. It's a lot, but it's necessary.

Next you have to install gstreamer. I had some trouble with this on the Dexter Industries image, but on the Wheezy it worked
right away.
```shell
sudo apt-get install gstreamer1.0
```

Then there's bluetooth to be set up. 
```shell
sudo apt-get install bluetooth bluez-utils blueman
```

I paired the BrickPi and the NXT using the Raspbian Desktop using blueman. It should also 
[be possible via the commandline.](http://www.heatxsink.com/entry/how-to-pair-a-bluetooth-device-from-command-line-on-linux)
Now you should configure a serial link to the NXT via rfcomm
```
sudo nano /etc/bluetooth/rfcomm.conf
```
Uncomment all the line the lines so you have a working rfcomm0 configuration there. Make sure you paste in the address of your
NXT device. You can find it by turning on the NXT and typing
```
hcitool scan
```
You can test if the serial connection works by typing:
```
sudo rfcomm connect rfcomm0
```

I used to do pip install jaraco.nxt for sending commands to the NXT, but I switched to nxt-python for sending commands
from the RPi to the NXT. nxt-python makes connecting and opening rfcomm channels easier. And it has support for an USB
connection to the brick, I should try that sometime.
```
sudo pip install nxt-python
sudo apt-get install python-bluez
```

Install Picamera for controlling the camera inside our python scripts
```
sudo pip install picamera
```

## On the Mac ##
It's easiest if you start by installing [homebrew](http://brew.sh). I'm a big fan. 

Next, get sdl2 and gstreamer:
```
brew install sdl2 gstreamer
```

Next get the needed python libraries. If you don't have (or want hg) you can also download the whole repo, of course.
```
hg clone https://bitbucket.org/marcusva/py-sdl2
cd py-sdl2
sudo python setup.py install
```

We might also need this [gst-python](http://gstreamer.freedesktop.org/src/gst-python/)