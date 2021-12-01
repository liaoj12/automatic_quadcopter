from SimpleCV import *

cam = Camera()
disp = Display()

while disp.isNotDone():
    img = cam.getImage()
    dist = img.colorDistance(Color.RED).dilate(2)
    segmented = dist.stretch(50, 120).invert()
    blobs = segmented.findBlobs()
    if disp.mouseLeft:
        img.save("img.png")
        break
    img.save(disp)
    
