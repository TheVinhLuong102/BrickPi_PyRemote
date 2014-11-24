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


# Usage #
Get the necessary dependencies, put the server script on the BrickPi and run the remote script on your mac.


# Installation #

## Raspberry Pi ##
Make sure you start with a fresh Raspbian image. I used wheezy. Don't use the image from dexter industries. BT doesn't 
  work on it and it has meager support for WiFi dongles.
First you'll have to install the BrickPi stuff. Clone their repository, install and follow the on-screen instructions
 like so:
 ```bash
 git clone https://github.com/DexterInd/BrickPi.git
 cd BrickPi/Setup Files/
 sudo chmod +x install.sh
 sudo ./install.sh
 cd ~
 git clone https://github.com/DexterInd/BrickPi_Python.git
 cd BrickPi_Python/
 python setup.py install
 ```
 
 the rest is TODO