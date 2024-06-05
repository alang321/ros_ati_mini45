import rospy
from force_sensor_ros_interface.msg import ForceMeasurement
import scripts.ft_interface as fif
import time
import std_msgs.msg
import scripts.preprocessing as pre
import numpy as np
import struct

import sys
import time

instrument = None

def force_sensor_interface():
    global instrument

    rospy.init_node('ros_ati_mini45', anonymous=True)

    #get parameters from launch file
    com_port = rospy.get_param('~com_port', '/dev/ttyUSB0')
    baudrate = rospy.get_param('~baudrate', 19200)

    rospy.loginfo("Port force sensor:" + com_port)
    rospy.loginfo("Baudrate:" + str(baudrate))

    pub_feedback = rospy.Publisher('force_measurements', ForceMeasurement, queue_size=10)

    instrument = fif.instrument_setup(com_port, baudrate)  # select USB port and baudrate (19200=100Hz / 115200=500Hz / 1250000=7000Hz)

    calib = fif.calibration(instrument)
    calib_mat = pre.calibration_matrix(calib)
    scaling_factors = pre.force_torque_scaling_factors(calib)
    rospy.loginfo("Got calibration")
    rospy.loginfo("Force sensor ready, publishing values")

    publish_values(pub_feedback, calib_mat, scaling_factors, instrument)

def publish_values(pub_feedback, calib_mat, scaling_factors, instrument):

    instrument._perform_command(70, b'U')

    while not rospy.is_shutdown():
        msg = ForceMeasurement()
        data = instrument.serial.read(13)
        if len(data) != 13 or (sum(data[:12]) & 0x7f) != (data[12] & 0x7f) or data[12] & 128:
            rospy.loginfo("Invalid sample received")
        else:
            gage_vector = np.array([struct.unpack('>h', data[j:j + 2])[0] for j in (0, 6, 2, 8, 4, 10)])
            values = (calib_mat @ gage_vector) / scaling_factors
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

if __name__ == '__main__':
    try:
        force_sensor_interface()
    except:
        sys.exit(0)