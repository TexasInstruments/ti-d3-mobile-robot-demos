#!/usr/bin/env python3

# ROS wrapper around dlp.py

import rospy
from std_msgs.msg import String
# from dlp import DLPDemo

from pyftdi.spi import SpiController, SpiIOError

class DLPDemo:

    # To add more messages, add new function message_dlp_<message_name> and update callback function

    # Video Name                                        Offset(StartAddr)	FrameCount
    # ----------------------------------------------------------------------------------
    # all                                               0x85940		258
    # go                                                0x1E27634		31
    # slow                                              0x2141744		93
    # stop                                              0x2E2942C		18
    # Correct values:
    #Video Name                                     Offset (0x)           Frame Count (decimal)
    #----------------------------------------------------------------------------------
    #Turn                                            85948                   39 = 0x27
    #Straight                                        591500                  45 = 0x2d
    #Go                                              A65AD8                  31 = 0x1f
    #Slow                                            D7FBE8                  93 = 0x5d
    #Stop                                            1A678D0                 18 = 0x12
    # ----------------------------------------------------------------------------------

    # message formats are:
    # CMD=0, Address (LSB), Address (MSB), Data (LSB), Data, Data, Data (MSB), Checksum
    # as a note, Address is register address, which is different from video address, which is Data to flash register address
    # more information can be found: https://www.ti.com/lit/ug/dlpu100/dlpu100.pdf?ts=1665158603108&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FDLP3021-Q1

    def __init__(self):
        self.ctrl = SpiController()
        # Configure the first interface (IF/1) of the FTDI device as a SPI master
        self.ctrl.configure('ftdi://ftdi:232h/1')

        # Get a port to a SPI slave
        self.spi = self.ctrl.get_port(0)
        self.spi.set_frequency(5E6)

    def message_dlp_go(self):
        """
        Display GO
        """
        #Go                                              A65AD8                  31 = 0x1f
        self.spi.exchange([0x00, 0x64, 0x00, 0xd8, 0x5a, 0xa6, 0x00, 0x3c]) # write video start address 1
        self.spi.exchange([0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00, 0x21]) # don't flip
        self.spi.exchange([0x00, 0x68, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x87]) # write video config 1
        self.spi.exchange([0x00, 0x74, 0x00, 0x15, 0x00, 0x00, 0x00, 0x89]) # write video control

    def message_dlp_slow(self):
        """
        Display SLOW
        """
        #Slow                                            D7FBE8                  93 = 0x5d
        self.spi.exchange([0x00, 0x64, 0x00, 0xE8, 0xFB, 0xD7, 0x00, 0x1E]) # write video start address 1
        self.spi.exchange([0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00, 0x21]) # don't flip
        self.spi.exchange([0x00, 0x68, 0x00, 0x5d, 0x00, 0x00, 0x00, 0xC5]) # write video config 1
        self.spi.exchange([0x00, 0x74, 0x00, 0x15, 0x00, 0x00, 0x00, 0x89]) # write video control

    def message_dlp_stop(self):
        """
        Display STOP
        """
        #Stop                                            1A678D0                 18 = 0x12
        self.spi.exchange([0x00, 0x64, 0x00, 0xd0, 0x78, 0xa6, 0x01, 0x53]) # write video start address 1
        self.spi.exchange([0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00, 0x21]) # don't flip
        self.spi.exchange([0x00, 0x68, 0x00, 0x12, 0x00, 0x00, 0x00, 0x7a]) # write video config 1
        self.spi.exchange([0x00, 0x74, 0x00, 0x15, 0x00, 0x00, 0x00, 0x89]) # write video control

    def message_dlp_turn(self):
        """
        Display TURN
        """
        #Turn                                            85948                   39 = 0x27
        self.spi.exchange([0x00, 0x64, 0x00, 0x48, 0x59, 0x08, 0x00, 0x0e]) # write video start address 1
        self.spi.exchange([0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00, 0x21]) # don't flip
        self.spi.exchange([0x00, 0x68, 0x00, 0x27, 0x00, 0x00, 0x00, 0x8f]) # write video config 1
        self.spi.exchange([0x00, 0x74, 0x00, 0x15, 0x00, 0x00, 0x00, 0x89]) # write video control

    def message_dlp_turn_flip(self, v_flip = False):
        """
        Display TURN
        """
        #Turn                                            85948                   39 = 0x27
        self.spi.exchange([0x00, 0x64, 0x00, 0x48, 0x59, 0x08, 0x00, 0x0d]) # write video start address 1
        self.spi.exchange([0x00, 0x68, 0x00, 0x27, 0x00, 0x00, 0x00, 0x8f]) # write video config 1
        if v_flip:
            self.spi.exchange([0x00, 0x20, 0x00, 0x11, 0x00, 0x00, 0x00, 0x31]) # flip
        else:
            self.spi.exchange([0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00, 0x21]) # don't flip

        self.spi.exchange([0x00, 0x74, 0x00, 0x15, 0x00, 0x00, 0x00, 0x89]) # write video control

    #def message_dlp_all(self):
        #"""
        #Display GO, then SLOW, then STOP in a loop
        #"""
        #self.spi.exchange([0x00, 0x64, 0x00, 0x40, 0x59, 0x08, 0x00, 0x05]) # write video start address 1
        #self.spi.exchange([0x00, 0x68, 0x00, 0x02, 0x01, 0x00, 0x00, 0x6b]) # write video config 1
        #self.spi.exchange([0x00, 0x74, 0x00, 0x15, 0x00, 0x00, 0x00, 0x89]) # write video control

    def disable_dlp_message(self):
        """
        Disable video
        """
        self.spi.exchange([0x00, 0x74, 0x00, 0x02, 0x00, 0x00, 0x00, 0x76]) # write video control

    def callback(self, data):
        if(data.data == 'stop'):
            self.message_dlp_stop()
        elif(data.data == 'slow'):
            self.message_dlp_slow()
        elif(data.data == 'go'):
            self.message_dlp_go()
        elif(data.data == 'turn_left'):
            self.message_dlp_turn_flip(False)
        elif(data.data == 'turn_right'):
            self.message_dlp_turn_flip(True)
        else:
            self.disable_dlp_message()


if __name__ == '__main__':
    dlp_demo = DLPDemo()
    rospy.init_node('dlp_ros', anonymous=True)
    rospy.Subscriber('dlp_test_string', String, dlp_demo.callback)
    rospy.spin()

