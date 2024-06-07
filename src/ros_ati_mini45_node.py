import rospy
from ros_ati_mini45.msg import ForceMeasurement
from ros_ati_mini45.msg import ZeroingDuration
import scripts.ft_interface as fif
import time
import std_msgs.msg
import scripts.preprocessing as pre
import numpy as np
import struct
import math

import threading

import sys
import time

MAX_ZERO_DURATION = rospy.Duration.from_sec(20)

zero_flag_mutex = threading.Lock()
zero_flag = False
zeroing_duration = None

def force_sensor_interface():
    rospy.init_node('ros_ati_mini45', anonymous=True)

    #get parameters from launch file
    com_port = rospy.get_param('~com_port', '/dev/ttyUSB0')
    baudrate = rospy.get_param('~baudrate', 19200)

    rospy.loginfo("Port force sensor:" + com_port)
    rospy.loginfo("Baudrate:" + str(baudrate))

    pub_feedback = rospy.Publisher('force_measurements', ForceMeasurement, queue_size=10)

    rospy.loginfo("Trying to connect to ATI Mini45")
    instrument = fif.instrument_setup(com_port, baudrate)
    #instrument = fif.instrument_setup('/dev/ttyUSB0', 19200)
    rospy.loginfo("ATI Mini45 connected")

    calib = fif.calibration(instrument)
    calib_mat = pre.calibration_matrix(calib)
    scaling_factors = pre.force_torque_scaling_factors(calib)
    rospy.loginfo("Got calibration")
    rospy.loginfo("Force sensor ready, publishing values")

    rospy.Subscriber("zero_force_sensor", ZeroingDuration, callback_zeroing)

    publish_values(pub_feedback, calib_mat, scaling_factors, instrument)

def publish_values(pub_feedback, calib_mat, scaling_factors, instrument):
    global zero_flag
    global zero_flag_mutex
    global zeroing_duration

    zeroing_values = np.zeros((6), dtype=np.float32)

    zeroing_data = np.zeros((math.ceil(MAX_ZERO_DURATION.to_sec() * 7000), 6), dtype=np.float32)
    current_idx = None # the current idx of saved zeroing data, used during zeroing

    currently_zeroing = False

    instrument._perform_command(70, b'U')

    while not rospy.is_shutdown():
        with zero_flag_mutex:
            local_zero_flag = zero_flag
        
        if local_zero_flag and not currently_zeroing:
            with zero_flag_mutex:
                local_zeroing_duration = zeroing_duration
                zero_flag = False

            t1 = time.time() + local_zeroing_duration.to_sec()

            current_idx = 0
            currently_zeroing = True

        if currently_zeroing: 
            if time.time() > t1:
                rospy.loginfo("Finished zeroing")
                currently_zeroing = False
                zeroing_values = np.mean(zeroing_data[:(current_idx - 1)], axis=0)
        
        data = instrument.serial.read(13)
        if len(data) != 13 or (sum(data[:12]) & 0x7f) != (data[12] & 0x7f) or data[12] & 128:
            rospy.loginfo("Invalid sample received")
            instrument.serial.flush()
        else:
            gage_vector = np.array([struct.unpack('>h', data[j:j + 2])[0] for j in (0, 6, 2, 8, 4, 10)])
            values = (calib_mat @ gage_vector) / scaling_factors

            if currently_zeroing:
                zeroing_data[current_idx, :] = values
                current_idx += 1
            else:
                values -= zeroing_values
                msg = ForceMeasurement()
                msg.Fx = values[0]
                msg.Fy = values[1]
                msg.Fz = values[2]
                msg.Tx = values[3]
                msg.Ty = values[4]
                msg.Tz = values[5]

                msg.stamp = rospy.Time.now()

                pub_feedback.publish(msg)

    instrument.serial.write(b'jaaammmmminngg')

    time.sleep(1)

    # Status Word
    if instrument.read_register(0x001D):
        raise RuntimeError("Status word nonzero")
    else:
        pass
    pass

def callback_zeroing(duration):
    global zero_flag
    global zero_flag_mutex
    global zeroing_duration
    rospy.loginfo("Received request to start zeroing")

    duartion_ros = rospy.Duration.from_sec(duration.duration)
    if duartion_ros < MAX_ZERO_DURATION:
        rospy.loginfo("Accepted, zeroing for " + str(duration.duration) + " seconds")
        with zero_flag_mutex:
            zero_flag = True
            zeroing_duration = duartion_ros
    else:
        rospy.loginfo("Zeroing duration too long")


if __name__ == '__main__':
    try:
        force_sensor_interface()
    except:
        sys.exit(0)