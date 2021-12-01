from __future__ import division
from SimpleCV import *
cam = Camera()

# window size: 640 * 480
# window center: (320, 240) r_desired = 35 +- 2 <= r_tolerence
# sqrt((x_circle - 320)**2+(y_circle - 240)**2) <= d_tolerence = 5


#constants
Window_Length = 640
Window_Width = 480
r = 35
d_tolerence = 5
r_tolerence = 2
D_img = 370
D_desired = 1500 # unit: mm
R_actual = 32.5 # unit: mm

def roll(dx, radius):
    """
        Function that decides how much roll we want the flight to 
        go in x direction based on dx.
        Lt  - 1100
        C   - 1500
        Rt  - 1900
    """
    if abs(dx) > (Window_Length/2 - radius):
        if dx > 0:
            return 1900
        else:
            return 1100
    else:
        return 1500 + 400 / (Window_Length/2 - radius) * dx


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
        go in z direction based on radius.
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


def yaw():
    """
        Function that decides how much yaw we want the flight to
        go in orientation. (Don't really need it now)
    """
    pass

# (roll, pitch, throttle)


while True:
    img = cam.getImage().flipHorizontal()
    dist = img.colorDistance(Color.RED).dilate(2)
    segmented = dist.stretch(50, 120).invert()
    blobs = segmented.findBlobs()
    if blobs:
        circles = blobs.filter([b.isCircle(0.2) for b in blobs])
        if circles:
            img.drawCircle((circles[-1].x, circles[-1].y), circles[-1].radius(), Color.BLUE, 3)
            c = circles[-1]
            img.drawRectangle(c.x-c.radius(), c.y-c.radius(), c.radius()*2, c.radius()*2, Color.BLUE, 2)
            print ("(%d, %d, %d)" % (c.x, c.y, c.radius()) )
            d = sqrt((c.x-Window_Length/2)**2+(c.y-Window_Width/2)**2)
            if d - d_tolerence > 0:
                #print "tell the flight shift leftward or rightward"
                dx = c.x - Window_Length/2
                dy = c.y - Window_Width/2
                print "Roll: ", roll(dx, c.radius())
                print "Throttle: ", throttle(dy, c.radius())
                

            d = distance(c.radius())
            print "Actual Distance: ", d
            print "Pitch: ", pitch(d)

            #if abs(c.radius()) - r_tolerence < 0:
                #print "tell the flight move forward and get close"
    img.show()







# first while loop provides bb
# print "Done the first while"
# bb = (c.x-c.radius(), c.y-c.radius(), c.radius()*2, c.radius()*2)
# ts = img.track(img=img, bb=bb)
# while True:
#     img = cam.getImage().flipHorizontal()
#     ts = img.track("camshift", ts=ts)
#     ts.drawPath()
#     ts.drawBB(Color.BLUE)
#     ts[-1].image.show()









