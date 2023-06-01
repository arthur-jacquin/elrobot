# modules

## detect faces
import argparse
import imutils
import time
import io
import cv2
import json
import random
import zenoh
import binascii
import numpy as np

## send commands to the robot
import sys
from datetime import datetime
import argparse
import curses
import zenoh
import json
from pycdr import cdr
from pycdr.types import int8, int32, uint32, float64


# types for robot commands

@cdr
class Vector3:
    x: float64
    y: float64
    z: float64

@cdr
class Twist:
    linear: Vector3
    angular: Vector3

@cdr
class Time:
    sec: int32
    nanosec: uint32

@cdr
class Log:
    stamp: Time
    level: int8
    name: str
    msg: str
    file: str
    function: str
    line: uint32


# configuration

## init parser and other settings
parser = argparse.ArgumentParser(
    prog='detect',
    description='detect faces on a given zenoh topic')
parser.add_argument('-d', '--delay', type=float, default=0.05,
                    help='delay between each frame in seconds')
## zenoh configuration
parser.add_argument('-m', '--mode', type=str, choices=['peer', 'client'],
                    help='The zenoh session mode.')
parser.add_argument('-e', '--connect', type=str, metavar='ENDPOINT', action='append',
                    help='zenoh endpoints to connect to.')
parser.add_argument('-l', '--listen', type=str, metavar='ENDPOINT', action='append',
                    help='zenoh endpoints to listen on.')
parser.add_argument('-p', '--prefix', type=str, default='elrobot',
                    help='resources prefix')
parser.add_argument('-c', '--config', type=str, metavar='FILE',
                    help='A zenoh configuration file.')
## detection settings
parser.add_argument('-w', '--width', type=int, default=200,
                    help='width of the published faces')
parser.add_argument('-q', '--quality', type=int, default=95,
                    help='quality of the published faces (0 - 100)')
parser.add_argument('-A', '--cascade', type=str,
                    default='haarcascade_frontalface_default.xml',
                    help='path to the face cascade file')
# robot commands settings
parser.add_argument('--cmd_vel', dest='cmd_vel', type=str, default='rt/turtle1/cmd_vel',
                    help='The "cmd_vel" ROS2 topic.')
parser.add_argument('--rosout', dest='rosout', type=str, default='rt/rosout',
                    help='The "rosout" ROS2 topic.')
parser.add_argument('-a', '--angular_scale', dest='angular_scale', type=float, default='100.0',
                    help='The angular scale.')
parser.add_argument('-x', '--linear_scale', dest='linear_scale', type=float, default='10.0',
                    help='The linear scale.')

args = parser.parse_args()
conf = zenoh.config_from_file(args.config) if args.config is not None else zenoh.Config()
if args.mode is not None:
    conf.insert_json5(zenoh.config.MODE_KEY, json.dumps(args.mode))
if args.connect is not None:
    conf.insert_json5(zenoh.config.CONNECT_KEY, json.dumps(args.connect))
if args.listen is not None:
    conf.insert_json5(zenoh.config.LISTEN_KEY, json.dumps(args.listen))

# global variables

jpeg_opts = [int(cv2.IMWRITE_JPEG_QUALITY), args.quality]
cams = {}
detector = cv2.CascadeClassifier(args.cascade)
box = None
names = {}

# functions

def pub_twist(linear, angular):
    # print("Pub twist: {} - {}".format(linear, angular))
    t = Twist(linear=Vector3(x=linear, y=0.0, z=0.0),
              angular=Vector3(x=0.0, y=0.0, z=angular))
    z.put(args.cmd_vel, t.serialize())


# callbacks

def frames_listener(sample):
    cam = str(sample.key_expr).split('/')[-1]
    cams[cam] = bytes(sample.payload)

def update_name(sample):
    chunks = str(sample.key_expr).split('/')
    cam = chunks[-3]
    face = int(chunks[-2])
    name = sample.payload.decode('utf-8')
    if cam not in names:
        names[cam] = {}
    names[cam][face] = name
    print(f'[INFO] Detected person change: on cam #{cam}, face {face} is {name}')

def rosout_callback(sample):
    log = Log.deserialize(sample.payload)
    # print(f'[{log.stamp.sec}.{log.stamp.nanosec}] [{log.name}]: {log.msg}')


# main script

print('[INFO] Open zenoh session...')
zenoh.init_logger()
z = zenoh.open(conf)

print('[INFO] Declare subscriptions...')
sub_cam = z.declare_subscriber(args.prefix + '/cams/*', frames_listener)
sub_name = z.declare_subscriber(args.prefix + '/faces/*/*/name', update_name)
sub_ros = z.declare_subscriber(args.rosout, rosout_callback)

print('[INFO] Start detection...')
while True:
    for cam in list(cams):
        npImage = np.frombuffer(cams[cam], dtype=np.uint8)
        matImage = cv2.imdecode(npImage, 1)

        gray = cv2.cvtColor(matImage, cv2.COLOR_BGR2GRAY)

        rects = detector.detectMultiScale(gray, scaleFactor=1.1,
                                          minNeighbors=5, minSize=(30, 30),
                                          flags=cv2.CASCADE_SCALE_IMAGE)
        boxes = [(y, x + w, y + h, x) for (x, y, w, h) in rects]

        faces = zip(range(len(boxes)), sorted(boxes))

        for (i, (top, right, bottom, left)) in faces:
            # cut and resize to focus on face
            face = matImage[int(top):int(bottom),
                            int(left):int(right)]
            face = imutils.resize(face, width=args.width)
            _, jpeg = cv2.imencode('.jpg', face, jpeg_opts)

            # put the data over the network
            box = {'left': int(left), 'right': int(right),
                   'top': int(top), 'bottom': int(bottom)}
            z.put(f'{args.prefix}/faces/{cam}/{i}', jpeg.tobytes())
            z.put(f'{args.prefix}/faces/{cam}/{i}/box', box)

            # check if a person is identified
            if cam not in names or i not in names[cam]:
                name = None
            else:
                name = names[cam][i]

            # send commands to robot over zenoh
            nlin, nang = 0, 0
            if name:
                if name == 'arthur':
                    nlin = 1
                elif name == 'sacha':
                    nlin = -1
                elif name == 'nicolas':
                    nang = 1
                elif name == 'alejandra':
                    nang = -1
            else:
                height = bottom - top
                middle = (left + right) >> 1
                if middle - 250 > 75:
                    nang = -1
                elif middle - 250 < -75:
                    nang = 1
                elif height > 90:
                    nlin = 1
                elif height < 70:
                    nlin = -1
                print(f'[INFO] Box information: {height}\t{middle}\t{nlin}\t{nang}')
            pub_twist(nlin * args.linear_scale, nang * args.angular_scale)

    time.sleep(args.delay)

sub_cam.undeclare()
sub_name.undeclare()
sub_ros.undeclare()
z.close()
