# TH Koeln
# Robotic Lab
# Gumer Rodriguez
# Quadrotor control simulation - Graphics for attitude estimation
# Structure code extracted from "Rotating 3D Cube using Python and Pygame" project
# available in codentronix.com

import sys, math, pygame, time
from pygame.locals import *
from operator import itemgetter
import MPU9150
import madgwick
from pyquaternion import Quaternion
import numpy as np
# Create point class for 3D space
class Point3D:
    def __init__(self, x = 0, y = 0, z = 0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    def rotateX(self, angle):
        """ Rotates the point around the X axis by the given angle in radians. """
        rad = angle
        cosa = math.cos(rad)
        sina = math.sin(rad)
        y = self.y * cosa - self.z * sina
        z = self.y * sina + self.z * cosa
        return Point3D(self.x, y, z)
 
    def rotateY(self, angle):
        """ Rotates the point around the Y axis by the given angle in radians. """
        rad = angle
        cosa = math.cos(rad)
        sina = math.sin(rad)
        z = self.z * cosa - self.x * sina
        x = self.z * sina + self.x * cosa
        return Point3D(x, self.y, z)
 
    def rotateZ(self, angle):
        """ Rotates the point around the Z axis by the given angle in radians. """
        rad = angle
        cosa = math.cos(rad)
        sina = math.sin(rad)
        x = self.x * cosa - self.y * sina
        y = self.x * sina + self.y * cosa
        return Point3D(x, y, self.z)
 
    def project(self, win_width, win_height, fov, viewer_distance):
        """ Transforms this 3D point to 2D using a perspective projection. """
        factor = fov / (viewer_distance + self.z)
        x = self.x * factor + win_width / 2
        y = -self.y * factor + win_height / 2
        return Point3D(x, y, self.z)
 
class Simulation:
    def __init__(self, win_width = 540, win_height = 320):
        pygame.init()
        #Set the window characteristics
        self.screen = pygame.display.set_mode((win_width, win_height))
        pygame.display.set_caption("IMU testing")
 
        self.clock = pygame.time.Clock()
        #Create the eight vertices for a rectangular prism
        self.vertices = [
            Point3D(-1,-0.2,1),
            Point3D(1,-0.2,1),
            Point3D(1,-0.2,-1),
            Point3D(-1,-0.2,-1),
            Point3D(-1,0.2,1),
            Point3D(1,0.2,1),
            Point3D(1,0.2,-1),
            Point3D(-1,0.2,-1)
        ]
        
        # Define the vertices that compose each of the 6 faces. These numbers are
        # indices to the vertices list defined above.
        self.faces  = [(0,1,2,3),(1,5,6,2),(5,4,7,6),(4,0,3,7),(0,4,5,1),(3,2,6,7)]
 
        # Define colors for each face
        #                down                   up
        self.colors = [(0,0,255),(0,255,0),(0,0,255),(0,255,0),(255,255,0),(255,255,0)]
        
        # Sampling time
        self.sampTime = 0.02
        
        # Start the MPU9150 module (sets up devices)
        self.mpu9150 = MPU9150.MPU9150()
        self.mad = madgwick.MadgwickAHRS(self.sampTime,0.2)
 
    def run(self):
        """ Main Loop """
        nowP = time.time()
        while 1:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
            #Limit the speed of the program
            self.clock.tick(int(1/self.sampTime))
            self.screen.fill((0,32,0))
            
            # Read linear acceleration
            accel = self.mpu9150.readAccel()
            ax = accel['x']
            ay = accel['y']
            az = accel['z']
            
            #Read angular acceleration
            gyro = self.mpu9150.readGyro()
            gx = gyro['x']
            gy = gyro['y']
            gz = gyro['z']
            
            #Dps to Rps
            gx = math.radians(gx)
            gy = math.radians(gy)
            gz = math.radians(gz)
            
            #Read magnetometer
            mag = self.mpu9150.readMagnet()
            mx = mag['x']
            my = mag['y']
            mz = mag['z']
            
            #Apply madgwick filter
            #Acce & Gyro axes are different from magnetometer's ones
            self.mad.Update(gx, gy, gz, ax, ay, az, my, mx, -mz)
            #self.mad.Update2(gx, gy, gz, ax, ay, az)
            #self.mad.MahonyQuaternionUpdate(gx, gy, gz, ax, ay, az, my, mx, -mz)
            time.sleep(0.003)
            
            q1 = self.mad.getQuaternion()
            qa = Quaternion(q1[0],q1[1],q1[2],q1[3])
            rm = qa.rotation_matrix
            angles= self.mad.quaternion_to_euler_angle()
            
            #Changes to aligne IMU to the screen position
            pos_x = 0
            pos_y = 0
            pos_z = 0

            # It will hold transformed vertices.
            t = []

            for v in self.vertices:
                # Rotate the point around X axis, then around Y axis and around Z axis.
                vect = [v.x, v.z, v.y]
                rot = np.matmul(rm,vect)
                # Traslation
                aux_x = rot[0] + pos_x
                aux_y = rot[2] + pos_z
                aux_z = rot[1] + pos_y
                aux_point = Point3D(aux_x,aux_y,aux_z)
                # Transform the point from 3D to 2D
                p = aux_point.project(self.screen.get_width(), self.screen.get_height(), 256, 4)
                # Put the point in the list of transformed vertices
                t.append(p)
 
            # Calculate the average Z values of each face.
            avg_z = []
            i = 0
            for f in self.faces:
                z = (t[f[0]].z + t[f[1]].z + t[f[2]].z + t[f[3]].z)/4
                avg_z.append([i,z])
                i = i + 1

            # Draw the faces using the Painter's algorithm:
            # Distant faces are drawn before the closer ones.
            for tmp in sorted(avg_z,key=itemgetter(1),reverse=True):
                face_index = tmp[0]
                f = self.faces[face_index]
                pointlist = [(t[f[0]].x, t[f[0]].y), (t[f[1]].x, t[f[1]].y),
                             (t[f[1]].x, t[f[1]].y), (t[f[2]].x, t[f[2]].y),
                             (t[f[2]].x, t[f[2]].y), (t[f[3]].x, t[f[3]].y),
                             (t[f[3]].x, t[f[3]].y), (t[f[0]].x, t[f[0]].y)]
                pygame.draw.polygon(self.screen,self.colors[face_index],pointlist)
 
            pygame.display.flip()
            
            elapsed = time.time() - nowP
            if (elapsed > 0.5):
                print ("Phi = %3.2f, Theta = %3.2f, Psi = %3.2f" % angles)
                nowP = time.time()

 
if __name__ == "__main__":
    Simulation().run()