#TH Koeln
#Robotic Lab
#Gumer Rodriguez
#Quadrotor control simulation - Attitude estimation

import MPU9150
import madgwick
import time
import decimal
import math

#====================================================================================================
#Program to test Madgwick filter
#====================================================================================================

# Sampling time
t= 0.02
# Start the MPU9150 module (sets up devices)
mpu9150 = MPU9150.MPU9150()
# Create the madgwick filter
mad = madgwick.MadgwickAHRS(0.02,0.1)
# Print the initial conditions
angles = mad.quaternion_to_euler_angle()    
print ("Phi = %3.2f, Theta = %3.2f, Psi = %3.2f" % angles)

# Start sensing the IMU
start = time.time()
nowP = time.time()
while True:
    now = time.time()
    #Read linear acceletarion
    accel = mpu9150.readAccel()
    ax = accel['x']
    ay = accel['y']
    az = accel['z']
    #Read angular acceleration
    gyro = mpu9150.readGyro()
    gx = gyro['x']
    gy = gyro['y']
    gz = gyro['z']
    #Read magnetometer
    mag = mpu9150.readMagnet()
    mx = mag['x']
    my = mag['y']
    mz = mag['z']
    #Apply madgwick filter
    #mad.Update(gx, gy, gz, ax, ay, az, mx, my, mz)
    mad.Update2(gx, gy, gz, ax, ay, az)
    angles = mad.quaternion_to_euler_angle()
    #Print results
    #print ("Phi = %3.2f, Theta = %3.2f, Psi = %3.2f" % angles)
    #Wait for the next loop
    elapsed = time.time() - now
    while elapsed < t:
        elapsed = time.time() - now
        #print 'Waiting'
    elapsed = time.time() - nowP
    if (elapsed > 0.5):
        print ("Phi = %3.2f, Theta = %3.2f, Psi = %3.2f" % angles)
        nowP = time.time()
    # Finish sensing after 15 seconds
    if(time.time()-start > 15):
        flag = 0