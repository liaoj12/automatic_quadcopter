# codes combining quadtest.py and try2.py
# to make the quadcopter flies and tracing 
# red circle card

from __future__ import division
from SimpleCV import *
from pymavlink import mavutil
import time, sys

cam = Camera()
#disp = Display()
mav1 = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
mav1.wait_heartbeat()


# window size: 640 * 480
# window center: (320, 240) r_desired = 35 +- 2 <= r_tolerence
# sqrt((x_circle - 320)**2+(y_circle - 240)**2) <= d_tolerence = 5


#constants
Window_Length = 640
Window_Width = 480
r = 35
d_tolerence = 5
r_tolerence = 2
D_img = 550
D_desired = 1500 # unit: mm
R_actual = 32.5 # unit: mm
trigger = 65535
start = False

def roll(dx, radius):
   """
       Function that decides how much roll we want the flight to 
       go in x direction based on dx.
       Lt  - 1100
       C   - 1500
       Rt  - 1900
   """
   if abs(dx) > (Window_Length/2 - radius):
       if dx < 0:
           return 1900
       else:
           return 1100
   else:
       return 1500 - 400 / (Window_Length/2 - radius) * dx

def throttle(dy, radius):
    """
        Function that decides how much throttle want the flight to
        go in y direction based on dy.
        High    - 1900
        C       - 1500
        Low     - 900
    """
    if abs(dy) > (Window_Width/2 - radius):
        if dy < 0:
            return 900    # maximum throttle
        else:
            return 1900
    else:
        return 1500 + 400 / (Window_Width/2 - radius) * (-dy)


# have to measure the cirlce, using similar triangles to solve the equations
# D_img = 192.3 px, theta = atan(c.radius() / D_img) = atan(acutal_R / actual_D)
# R_actual = 32.5 mm, D_actual_min = 200 mm, we want the D_actual_max = 2000
# D_desired = 1500 mm

def pitch(D_actual):
    """
        Function that decides how much pitch we want the flight to
        go in z direction based on radius. It tries to keep the quadcopter
        remains 1500mm away from the object.
        Fw  - 1100
        C   - 1500
        Bw  - 1900
    """
    if D_actual > 2000:
        return 1100
    elif D_actual < 500:
        return 1900
    else:
        if D_actual >= 1500:
            return 1500 + 0.8 * (D_desired - D_actual)
        else:
            return  1500 + 0.4* (D_desired - D_actual) 


def distance(R_img):
	return D_img * R_actual / R_img

# a value of 65535 means do nothing to that channel
# ch[roll, pitch, throttle, Unused, Unused, Unused]
ch = [trigger] * 8

mav1.mav.request_data_stream_send(mav1.target_system,mav1.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL,10,1)
while True:
    start = False
    msg = mav1.recv_match(blocking=True)
    # check if the switch is on. If so, then do the calculation, else pass the command
    # back to people
    msg_type = msg.get_type()
    if msg_type == "BAD_DATA":
        if mavutil.all_printable(msg.data):
            sys.stdout.write(msg.data)
            sys.stdout.flush()
    elif msg_type == "RC_CHANNELS_RAW":
        if msg.chan6_raw <1500:
            start = True
            mav1.mav.request_data_stream_send(mav1.target_system,mav1.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL,1,0)
        elif msg.chan6_raw > 1500:
            start = False
    
    # calculation
    while start:
        try:
            img = cam.getImage().flipHorizontal()
        except:
            mav1.mav.rc_channels_override_send(mav1.target_system, mav1.target_component, 0, ch[1], ch[2],
                   ch[3], ch[4], ch[5], ch[6], ch[7])
            mav1.mav.request_data_stream_send(mav1.target_system,mav1.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL,1,1)
            break

        dist = img.colorDistance(Color.RED).dilate(2)
        segmented = dist.stretch(150, 250).invert()
        blobs = segmented.findBlobs()

        if blobs:
            circles = blobs.filter([b.isCircle(0.2) for b in blobs])

            if circles:
                c = circles[-1]
                d = sqrt((c.x-Window_Length/2)**2+(c.y-Window_Width/2)**2)
                if d - d_tolerence > 0:
                    dx = c.x - Window_Length/2
                    #dy = c.y - Window_Width/2
                    ch[0] = roll(dx, c.radius())                

                mav1.mav.rc_channels_override_send(mav1.target_system, mav1.target_component, ch[0], ch[1], ch[2],
                    ch[3], ch[4], ch[5], ch[6], ch[7])

        #if no circle
            else:
                mav1.mav.rc_channels_override_send(mav1.target_system, mav1.target_component, 0, 0, 0,
                       0, 0, 0, 0, 0)
                mav1.mav.request_data_stream_send(mav1.target_system,mav1.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL,1,1)
                break

        




