'''
Created on 08/04/2014

@author: jeanmachuca

SAMPLE CODE:

This script is a test for device connected to GPIO port in raspberry pi 

For test purpose:

Step 1:
Connect the TX pin of the fingerprint GT511C3 to RX in the GPIO

Step 2:
Connect the RX pin of the fingerprint GT511C3 to TX in the GPIO 

Step 3: 
Connect the VCC pin of the fingerprint GTC511C3 to VCC 3,3 in GPIO

Step 4: 
Connect the Ground pin of fingerprint GT511C3 to ground pin in GPIO


This may be works fine, if don't, try to change the fingerprint baud rate with baud_to_115200.py sample code


'''
import FPS, sys
import pdb

def LoopCaptureFinger(scanner):
    while not fps.IsPressFinger():  #verify if the finger is in the scan
        print 'Place your finger on scanner'
        FPS.delay(1)
    print 'Your finger is in the scan'

    print 'Capture Finger'
    fingerFLAG = False
    for i in range(3):
        print 'Keep Finger Pressed'
        if fps.CaptureFinger(False):
            fingerFLAG = True
            print 'Remove Finger'
            while fps.IsPressFinger():
                FPS.delay(1)
            break
        FPS.delay(1)

    if fingerFLAG:
        print 'Finger successfully captured'
    else:
        print 'Finger was not captured'
    


if __name__ == '__main__':
    fps =  FPS.FPS_GT511C3(device_name='/dev/ttyAMA0',baud=9600,timeout=1,is_com=False)
    #fps =  FPS.FPS_GT511C3(device_name='/dev/serial0',baud=9600,timeout=2,is_com=False)

    fps.UseSerialDebug = False
    
#    fps.SetLED(True) # Turns ON the CMOS LED
#    print 'LED ON'
#    FPS.delay(3) # wait 1 second
#    fps.SetLED(False) # Turns ON the CMOS LED
#    print 'LED OFF'
#    FPS.delay(2)
#    fps.SetLED(True) # Turns ON the CMOS LED
#    print 'LED ON'
#    FPS.delay(3) # wait 1 second
#    fps.SetLED(False) # Turns ON the CMOS LED
#    print 'LED OFF'
#    FPS.delay(2)
#    fps.Close

    # Turn on LED
    print 'LED ON'
    fps.SetLED(False) # Turns ON the CMOS LED
    fps.SetLED(True) # Turns ON the CMOS LED
    fps.IsPressFinger() # Check if finger is in the scan because sometimes
                        #  the IfPressFinger command gives false positive
                        #  when LED is first turned on
    print 'Number Enrolled: %i'%(fps.GetEnrollCount())
    print 'Check if index 0 is enrolled'
    isEnrolled = fps.CheckEnrolled(0)
    if not isEnrolled:
        print 'Index 0 is open for enrollment'
    else:
        print 'Index 0 is alread enrolled'



    LoopCaptureFinger(fps)


    print 'Put your finger in the scan'
    FPS.delay(1) # wait 3 second
    counter = 0 # simple counter for wait 10 seconds
    while counter < 10:
        if fps.IsPressFinger():

            FPS.delay(1)
            fps.SetLED(False) # Turns OFF the CMOS LED
            FPS.delay(1)
            break
        else:
            FPS.delay(1) #wait 1 second
            print 'Put your finger in the scan'
            counter = counter + 1


    fps.Close() # Closes serial connection
    pass

