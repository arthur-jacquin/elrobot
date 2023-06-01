# subscribe

import sys
import time
from datetime import datetime
import argparse
import json
import zenoh
from zenoh import Reliability, Sample

# recognise faces

import argparse
import time
import io
import ast
import cv2
import json
import numpy as np
import face_recognition
import zenoh

# send commands to the robot

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

parser = argparse.ArgumentParser(
    prog='command_robot',
    description='face detector communicating commands to robot over zenoh')
parser.add_argument('-r', '--recognise', type=str, choices=['yes', 'no'], default='no',
                    help='If recognise is activated.')
# zenoh communications
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
parser.add_argument('-d', '--delay', type=float, default=0.05,
                    help='delay between each frame in seconds')
# robot commands
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
# recognition
reco = (args.recognise == 'yes')
position = {}
data = {}
data['encodings'] = []
data['names'] = []
cams = {}
done = -1
count = 0
# robot
cmd_vel = args.cmd_vel
rosout = args.rosout


# functions

def pub_twist(linear, angular):
    # print("Pub twist: {} - {}".format(linear, angular))
    t = Twist(linear=Vector3(x=linear, y=0.0, z=0.0),
              angular=Vector3(x=0.0, y=0.0, z=angular))
    z.put(cmd_vel, t.serialize())

def add_face_to_data(fdata, key, value):
    chunks = key.split('/')
    name = chunks[-2]
    num = chunks[-1]
    # print('[INFO] Add face to recognize: {}/{}'.format(name, num))
    fdata['names'].append(name)
    a = ast.literal_eval(value)
    fdata['encodings'].append(a)

# callbacks

def listener(sample: Sample):
    global position, count

    sliced = str(sample.key_expr).split('/')
    cam = sliced[-3]
    i = sliced[-2]
    position = json.loads(sample.payload.decode('utf-8'))
    count += 1
    #print(f"cam @{cam}, face id #{i}, left: {data['left']}")

def update_face_data(sample):
    if sample.kind == zenoh.SampleKind.PUT():
        add_face_to_data(data, str(sample.key_expr), sample.payload.decode("utf-8"))

def faces_listener(sample):
    # print('[DEBUG] Received face to recognize: {}'.format(sample.key_expr))
    chunks = str(sample.key_expr).split('/')
    cam = chunks[-2]
    face = int(chunks[-1])
    if cam not in cams:
        cams[cam] = {}
    cams[cam][face] = bytes(sample.payload)

def rosout_callback(sample):
    log = Log.deserialize(sample.payload)
    # print(f'[{log.stamp.sec}.{log.stamp.nanosec}] [{log.name}]: {log.msg}')


# code

print(reco)

# open session
print('[INFO] Open zenoh session...')
zenoh.init_logger()
z = zenoh.open(conf)

# get face vectors
if reco:
    print('[INFO] Retrieve faces vectors...')
    for vector in z.get(args.prefix + '/vectors/**', zenoh.ListCollector())():
        add_face_to_data(data, str(vector.data.key_expr), vector.data.payload.decode("utf-8"))

# suscriptions
print('[INFO] Declare zenoh subscriptions...')
sub_rects = z.declare_subscriber(args.prefix + '/faces/*/*/box', listener,
        reliability=Reliability.RELIABLE())
sub_ros = z.declare_subscriber(rosout, rosout_callback)
sub1 = z.declare_subscriber(args.prefix + '/vectors/**', update_face_data)
sub2 = z.declare_subscriber(args.prefix + '/faces/*/*', faces_listener)


# main loop
print("[INFO] Start detection...")
while True:
    for cam in list(cams):
        faces = cams[cam]
        for face in list(faces):
            name = 'Unknown'
            if reco:
                npImage = np.frombuffer(faces[face], dtype=np.uint8)
                matImage = cv2.imdecode(npImage, 1)
                rgb = cv2.cvtColor(matImage, cv2.COLOR_BGR2RGB)

                encodings = face_recognition.face_encodings(rgb)

                if len(encodings) > 0:
                    matches = face_recognition.compare_faces(data['encodings'],
                                                             encodings[0])
                    if True in matches:
                        matchedIdxs = [i for (i, b) in enumerate(matches) if b]
                        counts = {}
                        for i in matchedIdxs:
                            name = data['names'][i]
                            counts[name] = counts.get(name, 0) + 1
                        name = max(counts, key=counts.get)

            # z.put(args.prefix + f'/faces/{cam}/{face}/name', name)

            nlin, nang = 0, 0
            if name == 'arthur':
                nlin = 1
            elif name == 'sacha':
                nlin = -1
            elif name == 'nicolas':
                nang = 1
            elif name == 'alejandra':
                nang = -1
            elif done != count:
                done = count
                # print(position)
                height = position['bottom'] - position['top']
                off_axis = ((position['left'] + position['right']) >> 1) - 250

                if off_axis > 75:
                    nang = -1
                elif off_axis < -75:
                    nang = 1
                elif height > 90:
                    nlin = 1
                elif height < 70:
                    nlin = -1
            
                print(f'{height}\t{off_axis}\t{nlin}\t{nang}')

            pub_twist(nlin * args.linear_scale, nang * args.angular_scale)

    time.sleep(args.delay)

# closing connections
sub_rects.undeclare()
sub_ros.undeclare()
sub1.undeclare()
sub2.undeclare()
z.close()
