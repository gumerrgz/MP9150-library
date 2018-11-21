#TH Koeln
#Robotic Lab
#Gumer Rodriguez
#Magnetometer calibration

import MPU9150
import time
import sys
import decimal
import math
import matplotlib.pyplot as plt

#====================================================================================================
#Program to calibrate the MPU9150 magnetometer
#====================================================================================================

# Start the MPU9150 module (sets up devices)
mpu9150 = MPU9150.MPU9150()

#Run this program 4 times rotating the sensor 180 degrees each time
#Just use 2 axis
#python calibration.py -n 100 50
#samples = 100
#sample time = 50 ms
if len(sys.argv) >= 3 and sys.argv[1].find("-n") == 0:
    n = decimal.Decimal(sys.argv[2])
    dlay = 0
    if len(sys.argv) >= 4:
        dlay = decimal.Decimal(sys.argv[3])
    print("Taking %d samples delay %d mS" % (n, dlay))
    fdlay = 0.0
    fdlay = dlay / 1000
    i = 0
    samples = []
    while n > 0:
        mag = mpu9150.readMagnet()
        mX = mag['x']
        mY = mag['y']
        mZ = mag['z']
	print("mX %3.2f, mY %3.2f, mZ %3.2f" % (mX, mY, mZ))
	samples = samples + [[mX, mY, mZ]]
	i = i + 1
	n = n - 1
	time.sleep(fdlay)
    MX = 0.0
    MY = 0.0
    MZ = 0.0
    MX2 = 0.0
    MY2 = 0.0
    MZ2 = 0.0
    for (mX, mY, mZ) in samples:
        MX = MX + mX
	MY = MY + mY
	MZ = MZ + mZ
    MXAve = MX / i
    MYAve = MY / i
    MZAve = MZ / i
    for (mX, mY, mZ) in samples:
	MX2 = MX2 + (mX - MXAve) ** 2
	MY2 = MY2 + (mY - MYAve) ** 2
	MZ2 = MZ2 + (mZ - MZAve) ** 2

    print("Write this values")
    print("    Avg   StdDev")
    print("mX %6.1f %3.1f" % (MX / i, math.sqrt(MX2 / i)))
    print("mY %6.1f %3.1f" % (MY / i, math.sqrt(MY2 / i)))
    print("mZ %6.1f %3.1f" % (MZ / i, math.sqrt(MZ2 / i)))

#Without parameters this program graphics mx vs my
#Rotate the sensor using the z axis
#In theory you should see a perfect circle, if the calibration is good
#Otherwise, something like a circule
else:
    x = []
    y = []
    flag=1
    i = 0
    print "Tomando mediciones"
    while flag:
        accel = mpu9150.readAccel()
        #print " ax = " , ( accel['x'] )
        #print " ay = " , ( accel['y'] )
        #print " az = " , ( accel['z'] )

        gyro = mpu9150.readGyro()
        #print " gx = " , ( gyro['x'] )
        #print " gy = " , ( gyro['y'] )
        #print " gz = " , ( gyro['z'] )

        mag = mpu9150.readMagnet()
        print " mx = " , ( mag['x'] )
        print " my = " , ( mag['y'] )
        print " mz = " , ( mag['z'] )
        
        x.append(mag['x'])
        y.append(mag['y'])
        #Take 100 samples
        if (i>100):
            flag=0
        i=i+1
        time.sleep(0.1)

print ("Graphing")
plt.plot(x,y)
plt.xlabel("x flux")
plt.ylabel("y flux")
plt.show()