#TH Koeln
#Robotic Lab
#Gumer Rodriguez
#Implements Madgwicks AHRS and IMU algorithms in Python
#from the open source project in C developed by xioTechnologies
#/// See: http://x-io.co.uk/open-source-ahrs-with-x-imu/

import math

#====================================================================================================
#Class to apply Madgwick filter
#====================================================================================================

class MadgwickAHRS:
    #Get and set
    def getQuaternion(self):
        return self.Quaternion[0], self.Quaternion[1], self.Quaternion[2],self.Quaternion[3]

    def setQuaternion(self, param):
        self.Quaternion = param

    def __init__(self, sP, beta=1):
        """
        Construct for a new filter

        :param sP: time between two samples
        :param beta: filter constant
        :return: returns nothing
        """
        self.SamplePeriod = float(sP)
        self.Beta = float(beta)
        self.Quaternion = [1.0,0.0,0.0,0.0]
        self.Ki=0      #just for Mahony
        self.Kp=10     #just for Mahony
        self.eInt=[0.0,0.0,0.0]

    #gx gyroscope x axis measurement in radians/s
    #gy gyroscope y axis measurement in radians/s
    #gz gyroscope z axis measurement in radians/s
    #ax accelerometer x axis measurement in any calibrated units
    #ay accelerometer y axis measurement in any calibrated units
    #az accelerometer z axis measurement in any calibrated units
    #mx magnetometer x axis measurement in any calibrated units
    #my magnetometer y axis measurement in any calibrated units
    #mz magnetometer z axis measurement in any calibrated units
    
    #Complete Madgwick filter
    def Update(self, gx, gy, gz, ax, ay, az, mx, my, mz):
        q1 = self.Quaternion[0]
        q2 = self.Quaternion[1]
        q3 = self.Quaternion[2]
        q4 = self.Quaternion[3]

        #/ Auxiliary variables to avoid repeated arithmetic
        _2q1 = 2 * q1
        _2q2 = 2 * q2
        _2q3 = 2 * q3
        _2q4 = 2 * q4
        _2q1q3 = 2 * q1 * q3
        _2q3q4 = 2 * q3 * q4
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4

        #// Normalise accelerometer measurement
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if (norm == 0.0):
            return #// handle NaN
        norm = 1 / norm        #// use reciprocal for division
        ax *= norm
        ay *= norm
        az *= norm

        #// Normalise magnetometer measurement
        norm = math.sqrt(mx * mx + my * my + mz * mz)
        if (norm == 0.0):
            return #// handle NaN
        norm = 1 / norm        #// use reciprocal for division
        mx *= norm
        my *= norm
        mz *= norm

        #// Reference direction of Earth's magnetic field
        _2q1mx = 2 * q1 * mx
        _2q1my = 2 * q1 * my
        _2q1mz = 2 * q1 * mz
        _2q2mx = 2 * q2 * mx
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4
        _2bx = math.sqrt(hx * hx + hy * hy)
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4
        _4bx = 2 * _2bx
        _4bz = 2 * _2bz

        #// Gradient decent algorithm corrective step
        s1 = -_2q3 * (2 * q2q4 - _2q1q3 - ax) + _2q2 * (2 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s2 = _2q4 * (2 * q2q4 - _2q1q3 - ax) + _2q1 * (2 * q1q2 + _2q3q4 - ay) - 4 * q2 * (1 - 2 * q2q2 - 2 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s3 = -_2q1 * (2 * q2q4 - _2q1q3 - ax) + _2q4 * (2 * q1q2 + _2q3q4 - ay) - 4 * q3 * (1 - 2 * q2q2 - 2 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s4 = _2q2 * (2 * q2q4 - _2q1q3 - ax) + _2q3 * (2 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        norm = 1 / math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    #// normalise step magnitude
        s1 *= norm
        s2 *= norm
        s3 *= norm
        s4 *= norm

        #// Compute rate of change of quaternion
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.Beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.Beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.Beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.Beta * s4

        #// Integrate to yield quaternion
        q1 += qDot1 * self.SamplePeriod
        q2 += qDot2 * self.SamplePeriod
        q3 += qDot3 * self.SamplePeriod
        q4 += qDot4 * self.SamplePeriod
        norm = 1 / math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)    #// normalise quaternion
        self.Quaternion[0] = q1 * norm
        self.Quaternion[1] = q2 * norm
        self.Quaternion[2] = q3 * norm
        self.Quaternion[3] = q4 * norm

    #Without magnetometer Madgwick filter
    def Update2(self, gx, gy, gz, ax, ay, az):
        q1 = self.Quaternion[0]
        q2 = self.Quaternion[1]
        q3 = self.Quaternion[2]
        q4 = self.Quaternion[3]   #// short name local variable for readability

        #// Auxiliary variables to avoid repeated arithmetic
        _2q1 = 2 * q1
        _2q2 = 2 * q2
        _2q3 = 2 * q3
        _2q4 = 2 * q4
        _4q1 = 4 * q1
        _4q2 = 4 * q2
        _4q3 = 4 * q3
        _8q2 = 8 * q2
        _8q3 = 8 * q3
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3
        q4q4 = q4 * q4

        #// Normalise accelerometer measurement
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if (norm == 0.0):
            return #// handle NaN
        norm = 1 / norm        #// use reciprocal for division
        ax *= norm
        ay *= norm
        az *= norm

        #// Gradient decent algorithm corrective step
        s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay
        s2 = _4q2 * q4q4 - _2q4 * ax + 4 * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az
        s3 = 4 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az
        s4 = 4 * q2q2 * q4 - _2q2 * ax + 4 * q3q3 * q4 - _2q3 * ay
        norm = 1 / math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)    #// normalise step magnitude
        s1 *= norm
        s2 *= norm
        s3 *= norm
        s4 *= norm

        #// Compute rate of change of quaternion
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.Beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.Beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.Beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.Beta * s4

        #// Integrate to yield quaternion
        q1 += qDot1 * self.SamplePeriod
        q2 += qDot2 * self.SamplePeriod
        q3 += qDot3 * self.SamplePeriod
        q4 += qDot4 * self.SamplePeriod
        norm = 1 / math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)    #// normalise quaternion
        self.Quaternion[0] = q1 * norm
        self.Quaternion[1] = q2 * norm
        self.Quaternion[2] = q3 * norm
        self.Quaternion[3] = q4 * norm

    #Mahony - Madgwick filter
    def MahonyQuaternionUpdate(self, gx, gy, gz, ax, ay, az, mx, my, mz):
        q1 = self.Quaternion[0]
        q2 = self.Quaternion[1]
        q3 = self.Quaternion[2]
        q4 = self.Quaternion[3]

        # Auxiliary variables to avoid repeated arithmetic
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4   

        # Normalise accelerometer measurement
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if (norm == 0.0):
            return # handle NaN
        norm = 1.0 / norm        # use reciprocal for division
        ax *= norm
        ay *= norm
        az *= norm

        # Normalise magnetometer measurement
        norm = math.sqrt(mx * mx + my * my + mz * mz)
        if (norm == 0.0):
            return # handle NaN
        norm = 1.0 / norm        # use reciprocal for division
        mx *= norm
        my *= norm
        mz *= norm

        # Reference direction of Earth's magnetic field
        hx = 2.0 * mx * (0.5 - q3q3 - q4q4) + 2.0 * my * (q2q3 - q1q4) + 2.0 * mz * (q2q4 + q1q3)
        hy = 2.0 * mx * (q2q3 + q1q4) + 2.0 * my * (0.5 - q2q2 - q4q4) + 2.0 * mz * (q3q4 - q1q2)
        bx = math.sqrt((hx * hx) + (hy * hy))
        bz = 2.0 * mx * (q2q4 - q1q3) + 2.0 * my * (q3q4 + q1q2) + 2.0 * mz * (0.5 - q2q2 - q3q3)

        #Estimated direction of gravity and magnetic field
        vx = 2.0 * (q2q4 - q1q3)
        vy = 2.0 * (q1q2 + q3q4)
        vz = q1q1 - q2q2 - q3q3 + q4q4
        wx = 2.0 * bx * (0.5 - q3q3 - q4q4) + 2.0 * bz * (q2q4 - q1q3)
        wy = 2.0 * bx * (q2q3 - q1q4) + 2.0 * bz * (q1q2 + q3q4)
        wz = 2.0 * bx * (q1q3 + q2q4) + 2.0 * bz * (0.5 - q2q2 - q3q3)  

        # Error is cross product between estimated direction and measured direction of gravity
        ex = (ay * vz - az * vy) + (my * wz - mz * wy)
        ey = (az * vx - ax * vz) + (mz * wx - mx * wz)
        ez = (ax * vy - ay * vx) + (mx * wy - my * wx)

        if (self.Ki > 0.0):
            self.eInt[0] += ex      # accumulate integral error
            self.eInt[1] += ey
            self.eInt[2] += ez
        else:
            self.eInt[0] = 0.0     # prevent integral wind up
            self.eInt[1] = 0.0
            self.eInt[2] = 0.0

        # Apply feedback terms
        gx = gx + self.Kp * ex + self.Ki * self.eInt[0]
        gy = gy + self.Kp * ey + self.Ki * self.eInt[1]
        gz = gz + self.Kp * ez + self.Ki * self.eInt[2]

        # Integrate rate of change of quaternion
        pa = q2
        pb = q3
        pc = q4
        q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5 * self.SamplePeriod)
        q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5 *  self.SamplePeriod)
        q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5 *  self.SamplePeriod)
        q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5 *  self.SamplePeriod)

        # Normalise quaternion
        norm = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
        norm = 1.0 / norm
        self.Quaternion[0] = q1 * norm
        self.Quaternion[1] = q2 * norm
        self.Quaternion[2] = q3 * norm
        self.Quaternion[3] = q4 * norm

    def quaternion_to_euler_angle(self):
        w = self.Quaternion[0]
        x = self.Quaternion[1]
        y = self.Quaternion[2]
        z = self.Quaternion[3]
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        #X = math.degrees(math.atan2(t0, t1))
        X = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        #Y = math.degrees(math.asin(t2))
        Y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        #Z = math.degrees(math.atan2(t3, t4))
        Z = math.atan2(t3, t4)

        return X, Y, Z