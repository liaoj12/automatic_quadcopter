# -*- coding: utf-8 -*-
"""
Created on Thu Apr 16 09:08:46 2015

@author: martin
"""

from pymavlink import mavutil, mavextra, mavparm
import time

# a value of 65535 means do nothing to that channel
# ch[roll, pitch, throttle, x, x, x]
ch = [65535, 1000, 65535, 65535, 65535, 65535, 65535, 65535]

mav1 = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
mav1.wait_heartbeat()
print "Connected to system",mav1.target_system, " component",mav1.target_component

##mav1.mav.request_data_stream_send(mav1.target_system,mav1.target_component,mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,1,1)
##i = 0
while True:
    msg = mav1.recv_match(blocking=True)
    if msg.fieldnames[0] == 'time_boot_ms' and msg.fieldnames[1] == 'port':
        print msg.chan6_raw



##
##
##mav1.mav.rc_channels_override_send(mav1.target_system,mav1.target_component, ch[0], ch[1], ch[2],
##                                   ch[3],ch[4],ch[5],ch[6],ch[7])
##
### a value of zero returns control of that channel to the transmitter
##ch = [65535, 0,65535,65535,65535,65535,65535,65535]
##time.sleep(3)
##
##mav1.mav.rc_channels_override_send(mav1.target_system,mav1.target_component, ch[0], ch[1], ch[2],
##                                   ch[3],ch[4],ch[5],ch[6],ch[7])
##
##while True:
##    mav1.wait_heartbeat()
##    print time.time(), mav1.flightmode

