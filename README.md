# MP9150-library
This project was made in order to use the MP9150 sensor with a Raspberry Pi using Python

## Getting started

Download all the files in your computer

### Prerequisites

Python 2.7 and Pygame installed

### Core files

* MPU9150.py - this file will let you use the MPU9150 sensor
* madgwick.py - this file implements the madgwick filter

### Runnable files

* RTgraphicsIMU.py - creates a graphic simulation that mimics the MPU-9150 attitude
* calibration.py - is used to calibrate the sensor

### Calibrating the sensor
It is important to emphasize that the sensor needs to be calibrated first to get the correct measurements. Both, the gyroscope and the magnetometer have little offsets that need to be taken into consideration.  

To get the gyroscope offsets, it is necessary to fix the sensor and calculate the average of multiple measurements.  

In order to get the magnetometer offsets it is necessary to calculate the average of the averages of the multiple measurements, this is possible by fixing the sensor in four different positions, performing a 180 degree turn from the last sensing using only two axes.  

It is necessary to create a file named MPUoffsets.txt to store the averages of the offsets with the next structure:  

gx gy gz  
mx my mz  

Where gx, gy and gz are the gyroscopes offsets (float); and mx, my and mz are the magnetometer offsets (int). They are separated with spaces.  
  
A more detailed procedure is localized on the Simple Manual Magnetometer Calibration (https://www.instructables.com/id/Simple-Manual-Magnetometer-Calibration/).  

You can use the files calibration.py and calculateAttitude.py.

## Authors

* Gumer Rodriguez 
* This project was developed in the Robotics Lab of Cologne University of Applied Sciences

## Acknowledgments

* MPU9150.py was created editing FaBo9Axis_MPU9250 library to fit the MPU9150 (https://github.com/FaBoPlatform/FaBo9AXIS-MPU9250-Python)
