#!/usr/bin/env python

from __future__ import print_function

import enum
import re
import struct
import sys
import threading
import time
import math
import serial
from serial.tools.list_ports import comports
from common import *
import rospy
from std_msgs.msg import String, UInt8, Header, MultiArrayLayout, MultiArrayDimension, Float64MultiArray
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu
from ros_myo.msg import MyoArm, EmgArray

def multichr(ords):
    if sys.version_info[0] >= 3:
        return bytes(ords)
    else:
        return ''.join(map(chr, ords))

def multiord(b):
    if sys.version_info[0] >= 3:
        return list(b)
    else:
        return map(ord, b)

class Arm(enum.Enum):
    UNKNOWN = 0
    RIGHT = 1
    LEFT = 2

class XDirection(enum.Enum):
    UNKNOWN = 0
    X_TOWARD_WRIST = 1
    X_TOWARD_ELBOW = 2

class Pose(enum.Enum):
    REST = 0
    FIST = 1
    WAVE_IN = 2
    WAVE_OUT = 3
    FINGERS_SPREAD = 4
    THUMB_TO_PINKY = 5
    UNKNOWN = 255


class Packet(object):
    def __init__(self, ords):
        self.typ = ords[0]
        self.cls = ords[2]
        self.cmd = ords[3]
        self.payload = multichr(ords[4:])

    def __repr__(self):
        return 'Packet(%02X, %02X, %02X, [%s])' % \
            (self.typ, self.cls, self.cmd,
             ' '.join('%02X' % b for b in multiord(self.payload)))


class BT(object):
    '''Implements the non-Myo-specific details of the Bluetooth protocol.'''
    def __init__(self, tty):
        self.ser = serial.Serial(port=tty, baudrate=9600, dsrdtr=1)
        self.buf = []
        self.lock = threading.Lock()
        self.handlers = []

    ## internal data-handling methods
    def recv_packet(self, timeout=None):
        t0 = time.time()
        self.ser.timeout = None
        while timeout is None or time.time() < t0 + timeout:
            if timeout is not None: self.ser.timeout = t0 + timeout - time.time()
            c = self.ser.read()
            if not c:
                return None

            ret = self.proc_byte(ord(c))
            if ret:
                if ret.typ == 0x80:
                    self.handle_event(ret)
                return ret

    def recv_packets(self, timeout=.5):
        res = []
        t0 = time.time()
        while time.time() < t0 + timeout:
            p = self.recv_packet(t0 + timeout - time.time())
            if not p:
                return res
            res.append(p)
        return res

    def proc_byte(self, c):
        if not self.buf:
            if c in [0x00, 0x80, 0x08, 0x88]:
                self.buf.append(c)
            return None
        elif len(self.buf) == 1:
            self.buf.append(c)
            self.packet_len = 4 + (self.buf[0] & 0x07) + self.buf[1]
            return None
        else:
            self.buf.append(c)

        if self.packet_len and len(self.buf) == self.packet_len:
            p = Packet(self.buf)
            self.buf = []
            return p
        return None

    def handle_event(self, p):
        for h in self.handlers:
            h(p)

    def add_handler(self, h):
        self.handlers.append(h)

    def remove_handler(self, h):
        try: self.handlers.remove(h)
        except ValueError: pass

    def wait_event(self, cls, cmd):
        res = [None]
        def h(p):
            if p.cls == cls and p.cmd == cmd:
                res[0] = p
        self.add_handler(h)
        while res[0] is None:
            self.recv_packet()
        self.remove_handler(h)
        return res[0]

    ## specific BLE commands
    def connect(self, addr):
        return self.send_command(6, 3, pack('6sBHHHH', multichr(addr), 0, 6, 6, 64, 0))

    def get_connections(self):
        return self.send_command(0, 6)

    def discover(self):
        return self.send_command(6, 2, b'\x01')

    def end_scan(self):
        return self.send_command(6, 4)

    def disconnect(self, h):
        return self.send_command(3, 0, pack('B', h))

    def read_attr(self, con, attr):
        self.send_command(4, 4, pack('BH', con, attr))
        return self.wait_event(4, 5)

    def write_attr(self, con, attr, val):
        self.send_command(4, 5, pack('BHB', con, attr, len(val)) + val)
        return self.wait_event(4, 1)

    def send_command(self, cls, cmd, payload=b'', wait_resp=True):
        s = pack('4B', 0, len(payload), cls, cmd) + payload
        self.ser.write(s)

        while True:
            p = self.recv_packet()

            ## no timeout, so p won't be None
            if p.typ == 0:
                return p

            ## not a response: must be an event
            self.handle_event(p)


class MyoRaw(object):
    '''Implements the Myo-specific communication protocol.'''

    def __init__(self, tty=None):
        rospy.logdebug("Namespace:%s" % rospy.get_namespace())
        self.top_myo = rospy.get_param('/ros_myo/top')
        self.low_myo = rospy.get_param('/ros_myo/low')
        self.myo_port = rospy.get_param('ros_myo/port')
        if tty is None:
            tty = self.myo_port
        if tty is None:
            raise ValueError('Myo dongle not found!')

        self.bt = BT(tty)
        self.conn = None
        self.emg_handlers = []
        self.imu_handlers = []
        self.arm_handlers = []
        self.pose_handlers = []
        self.vibration_requested = None
        self.myo_name = ""

    # def detect_tty(self):
        # for p in comports():
        #     rospy.logdebug('ComPorts:', p[0], p[1], p[2])
        #     if re.search(r'PID=2458:0*1', p[2]):
        #         rospy.logdebug('using device:', p[0])
        #         return p[0]
        # return '/dev/ttyACM1'

    def run(self, timeout=None):
        self.bt.recv_packet(timeout)
        if self.vibration_requested:
            self.vibrate(self.vibration_requested)
            self.vibration_requested = None

    def connect(self):
        ## stop everything from before
        self.bt.end_scan()
        self.bt.disconnect(0)
        self.bt.disconnect(1)
        self.bt.disconnect(2)

        ## start scanning
        rospy.logdebug('scanning...')
        self.bt.discover()
        while True:
            p = self.bt.recv_packet()
            # rospy.logdebug('scan response:', p)

            if p.payload.endswith(b'\x06\x42\x48\x12\x4A\x7F\x2C\x48\x47\xB9\xDE\x04\xA9\x01\x00\x06\xD5'):
                addr = list(multiord(p.payload[2:8]))
                break
        self.bt.end_scan()

        ## connect and wait for status event
        conn_pkt = self.bt.connect(addr)
        self.conn = multiord(conn_pkt.payload)[-1]
        self.bt.wait_event(3, 0)

        ## get firmware version
        fw = self.read_attr(0x17)
        _, _, _, _, v0, v1, v2, v3 = unpack('BHBBHHHH', fw.payload)
        rospy.logdebug('firmware version: %d.%d.%d.%d' % (v0, v1, v2, v3))

        self.old = (v0 == 0)

        if self.old:
            ## don't know what these do; Myo Connect sends them, though we get data
            ## fine without them
            self.write_attr(0x19, b'\x01\x02\x00\x00')
            self.write_attr(0x2f, b'\x01\x00')
            self.write_attr(0x2c, b'\x01\x00')
            self.write_attr(0x32, b'\x01\x00')
            self.write_attr(0x35, b'\x01\x00')

            ## enable EMG data
            self.write_attr(0x28, b'\x01\x00')
            ## enable IMU data
            self.write_attr(0x1d, b'\x01\x00')

            ## Sampling rate of the underlying EMG sensor, capped to 1000. If it's
            ## less than 1000, emg_hz is correct. If it is greater, the actual
            ## framerate starts dropping inversely. Also, if this is much less than
            ## 1000, EMG data becomes slower to respond to changes. In conclusion,
            ## 1000 is probably a good value.
            C = 1000
            emg_hz = 50
            ## strength of low-pass filtering of EMG data
            emg_smooth = 100

            imu_hz = 50

            ## send sensor parameters, or we don't get any data
            self.write_attr(0x19, pack('BBBBHBBBBB', 2, 9, 2, 1, C, emg_smooth, C // emg_hz, imu_hz, 0, 0))

        else:
            name = self.read_attr(0x03)
            rospy.loginfo('device name:%s' % name.payload)
            self.myo_name = name.payload.split(' ', 2)[0] # Get Colour Name
            self.myo_name = self.myo_name.strip('\t\00\03\07')

            if (self.myo_name == self.top_myo):
                self.top_low = True
                self.myo_ns = '/top_myo/'
            elif (self.myo_name == self.low_myo):
                self.top_low = False
                self.myo_ns = '/low_myo/'
            else:
                rospy.logerr('ERROR: Myo %s was not set as top or low!' % self.myo_name)
                rospy.signal_shutdown('ERROR')


            ## enable IMU data
            self.write_attr(0x1d, b'\x01\x00')
            ## enable on/off arm notifications
            self.write_attr(0x24, b'\x02\x00')

            # self.write_attr(0x19, b'\x01\x03\x00\x01\x01')
            self.start_raw()

        ## add data handlers
        def handle_data(p):
            if (p.cls, p.cmd) != (4, 5): return

            c, attr, typ = unpack('BHB', p.payload[:4])
            pay = p.payload[5:]

            if attr == 0x27:
                vals = unpack('8HB', pay)
                ## not entirely sure what the last byte is, but it's a bitmask that
                ## seems to indicate which sensors think they're being moved around or
                ## something
                emg = vals[:8]
                moving = vals[8]
                self.on_emg(emg, moving)
            elif attr == 0x1c:
                vals = unpack('10h', pay)
                quat = vals[:4]
                acc = vals[4:7]
                gyro = vals[7:10]
                self.on_imu(quat, acc, gyro)
            elif attr == 0x23:
                # typ, val, xdir = unpack('3B', pay)
                typ, val, xdir, _, _, _ = unpack('6B', pay)

                if typ == 1: # on arm
                    self.on_arm(Arm(val), XDirection(xdir))
                elif typ == 2: # removed from arm
                    self.on_arm(Arm.UNKNOWN, XDirection.UNKNOWN)
                elif typ == 3: # pose
                    self.on_pose(Pose(val))
            else:
                rospy.logwarn('data with unknown attr: %02X %s' % (attr, p))

        self.bt.add_handler(handle_data)


    def write_attr(self, attr, val):
        if self.conn is not None:
            self.bt.write_attr(self.conn, attr, val)

    def read_attr(self, attr):
        if self.conn is not None:
            return self.bt.read_attr(self.conn, attr)
        return None

    def disconnect(self):
        if self.conn is not None:
            self.bt.disconnect(self.conn)

    def start_raw(self):
        '''Sending this sequence for v1.0 firmware seems to enable both raw data and
        pose notifications.
        '''

        self.write_attr(0x28, b'\x01\x00')
        self.write_attr(0x19, b'\x01\x03\x01\x01\x00')
        self.write_attr(0x19, b'\x01\x03\x01\x01\x01')

    def vibrate(self, length):
        # if length in xrange(1, 3):
            ## first byte tells it to vibrate; purpose of second byte is unknown
            # self.write_attr(0x19, pack('3B', 3, 1, length))

        self.write_attr(0x19, pack('20B', 7, 18, 0, 200, 255, 0, 200, 255, 0, 200, 255, 0, 200, 255, 0, 200, 255, 0, 200, 255))
        # self.write_attr(0x19, pack('3B', 0, 200, 255))
        # self.write_attr(0x19, pack('3B', 0, 200, 255))
        # self.write_attr(0x19, pack('3B', 0, 200, 255))
        # self.write_attr(0x19, pack('3B', 0, 200, 255))
        # self.write_attr(0x19, pack('3B', 0, 200, 255))
        # self.write_attr(0x19, pack('3B', 0, 200, 255))

    def set_sleep_mode(self, mode):
        if mode in [0, 1]:
            self.write_attr(0x19, pack('3B', 9, 1, mode))

    def add_emg_handler(self, h):
        self.emg_handlers.append(h)

    def add_imu_handler(self, h):
        self.imu_handlers.append(h)

    def add_pose_handler(self, h):
        self.pose_handlers.append(h)

    def add_arm_handler(self, h):
        self.arm_handlers.append(h)


    def on_emg(self, emg, moving):
        for h in self.emg_handlers:
            h(emg, moving)

    def on_imu(self, quat, acc, gyro):
        for h in self.imu_handlers:
            h(quat, acc, gyro)

    def on_pose(self, p):
        for h in self.pose_handlers:
            h(p)

    def on_arm(self, arm, xdir):
        for h in self.arm_handlers:
            h(arm, xdir)

if __name__ == '__main__':
    rospy.init_node('raw', log_level=rospy.INFO)
    # Start by initializing the Myo and attempting to connect.
    # If no Myo is found, we attempt to reconnect every 0.5 seconds
    connected = 0;
    rospy.logdebug("Initializing...")
    while(connected == 0):
        try:
            m = MyoRaw()
            connected = 1;
        except (ValueError, KeyboardInterrupt) as e:
            rospy.logwarn("Myo Armband not found. Attempting to connect...")
            rospy.sleep(0.5)
            pass

    m.connect()
    m.set_sleep_mode(1)

    rospy.loginfo("Connecting to %s using %s as %s", m.myo_name, m.myo_port, m.myo_ns)

    # Define Publishers
    imuPub = rospy.Publisher(m.myo_ns + 'imu', Imu, queue_size=10)
    emgPub = rospy.Publisher(m.myo_ns + 'emg', EmgArray, queue_size=10)
    armPub = rospy.Publisher(m.myo_ns + 'arm', MyoArm, queue_size=10)
    gestPub = rospy.Publisher(m.myo_ns + 'gesture', UInt8, queue_size=10)


    # Package the EMG data into an EmgArray
    def proc_emg(emg, moving, times=[]):
        ## create an array of ints for emg data
        emgPub.publish(emg)

        ## print framerate of received data
        times.append(time.time())
        if len(times) > 20:
            #rospy.logwarn((len(times) - 1) / (times[-1] - times[0]))
            times.pop(0)
    # Package the IMU data into an Imu message
    def proc_imu(quat1, acc, gyro):
        # New info: https://github.com/thalmiclabs/myo-bluetooth/blob/master/myohw.h#L292-L295
        # Scale values for unpacking IMU data
        # define MYOHW_ORIENTATION_SCALE   16384.0f ///< See myohw_imu_data_t::orientation
        # define MYOHW_ACCELEROMETER_SCALE 2048.0f  ///< See myohw_imu_data_t::accelerometer
        # define MYOHW_GYROSCOPE_SCALE     16.0f    ///< See myohw_imu_data_t::gyroscope
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'myo'
        # We currently do not know the covariance of the sensors with each other
        cov = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        quat = Quaternion(quat1[0]/16384.0, quat1[1]/16384.0, quat1[2]/16384.0, quat1[3]/16384.0)
        ## Normalize the quaternion and accelerometer values
        quatNorm = math.sqrt(quat.x*quat.x+quat.y*quat.y+quat.z*quat.z+quat.w*quat.w)
        normQuat = Quaternion(quat.x/quatNorm, quat.y/quatNorm, quat.z/quatNorm, quat.w/quatNorm)
        normAcc = Vector3(acc[0]/2048.0, acc[1]/2048.0, acc[2]/2048.0)
        normGyro = Vector3(gyro[0]/16.0, gyro[1]/16.0, gyro[2]/16.0)
        imu = Imu(h, normQuat, cov, normGyro, cov, normAcc, cov)
        imuPub.publish(imu)

    # Package the arm and x-axis direction into an Arm message
    def proc_arm(arm, xdir):
        #When the arm state changes, publish the new arm and orientation
        calibArm=MyoArm(arm.value, xdir.value)
        armPub.publish(calibArm)

    # Publish the value of an enumerated gesture
    def proc_pose(p):
        gestPub.publish(p.value)

    m.add_emg_handler(proc_emg)
    m.add_imu_handler(proc_imu)
    m.add_arm_handler(proc_arm)
    m.add_pose_handler(proc_pose)


# Add a way to vibrate
    def vibrate_cb(data):
        rospy.logdebug("Received vibrate msg: " + str(data))
        # This will be checked on every m.run() call
        m.vibration_requested = data.data

    vibSubs = rospy.Subscriber(m.myo_ns + 'vib', UInt8, vibrate_cb, queue_size=10)

    try:
        while not rospy.is_shutdown():
            m.run(1)

    except (rospy.ROSInterruptException, serial.serialutil.SerialException) as e:
        pass
    finally:
        rospy.loginfo("Disconnecting...")
        m.disconnect()
