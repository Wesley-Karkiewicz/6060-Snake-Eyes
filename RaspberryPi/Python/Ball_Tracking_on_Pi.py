#!/usr/bin/env python3
#Wesley Karkiewicz--6060's Snake eyes
#
#
#
#This Frankenstein code is The demo Python script that came with
#the FRCVison image mashed with my ball tracking code. It Does
#Vision Processing exclusively on the Pi, should run faster
#then the Driver Station Version. Hopefully this script is just
#plug and play for you but if not you will need to edit tjhis script
#yourself to. work with your set up. I Have comments below on where
#you should edit if need be.
#
#
#
import json
import time
import sys
import numpy as np
import cv2
from cscore import CameraServer, VideoSource
from networktables import NetworkTablesInstance

#   JSON format:
#   {
#       "team": <team number>,
#       "ntmode": <"client" or "server", "client" if unspecified>
#       "cameras": [
#           {
#               "name": <camera name>
#               "path": <path, e.g. "/dev/video0">
#               "pixel format": <"MJPEG", "YUYV", etc>   // optional
#               "width": <video mode width>              // optional
#               "height": <video mode height>            // optional
#               "fps": <video mode fps>                  // optional
#               "brightness": <percentage brightness>    // optional
#               "white balance": <"auto", "hold", value> // optional
#               "exposure": <"auto", "hold", value>      // optional
#               "properties": [                          // optional
#                   {
#                       "name": <property name>
#                       "value": <property value>
#                   }
#               ]
#           }
#       ]
#   }

configFile = "/boot/frc.json"

class CameraConfig: pass

team = None
server = False
cameraConfigs = []

"""Report parse error."""
def parseError(str):
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

"""Read single camera configuration."""
def readCameraConfig(config):
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    cam.config = config

    cameraConfigs.append(cam)
    return True

"""Read configuration file."""
def readConfig():
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    return True




#This should be a class lowkey but it'll work
def TrackTheBall(frame, sd):
    #if no frame arrives, the vid is over or camera is unavalible
    if frame is None:
        sd.putNumber('GettingFrameData',False)
    else:
        sd.putNumber('GettingFrameData',True)

    
    frame = cv2.flip(frame, 0)
    cv2.line(frame, (0,0), (10,10), (255,255,255), 7)
    return frame


if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

    # start NetworkTables to send to smartDashboard
    ntinst = NetworkTablesInstance.getDefault()

    print("Setting up NetworkTables client for team {}".format(team))
    ntinst.startClientTeam(team)

    SmartDashBoardValues = ntinst.getTable('SmartDashBoard')
    
    #Start up camera stuff
    print("Connecting to camera")
    cs = CameraServer.getInstance()
    cs.enableLogging()
    camera = cs.startAutomaticCapture()
    print("connected")

    #This Is the object we pull the imgs for OpenCV magic
    CvSink = cs.getVideo()
    
    #This will send the process frames to the Driver station
    #allowing the us to see what OpenCV sees
    outputStream = cs.putVideo("Camera RPi Camera 0", 160,120)

    #buffer to store img data
    img = np.zeros(shape=(260,260,3), dtype=np.uint8)
    # loop forever
    while True:
        #Quick little FYI, This will throw a Unicode Deode Error first time around
        #Something about a invalid start byte. This is fine, the Program will continue
        # and after a few loops should start grabing frames from the camera
        GotFrame, img = CvSink.grabFrame(img)
        if GotFrame  == 0:
            outputStream.notifyError(CvSink.getError())
            continue
        print("yeet")
        cv2.line(img, (0,0), (10,10), (255,255,255), 7)
        outputStream.putFrame(img)


