#TH Koeln
#Robotic Lab
#Gumer Rodriguez
#Edited from FaBo9Axis_MPU9250 library to fit the MPU9150
# See: https://github.com/FaBoPlatform/FaBo9AXIS-MPU9250-Python

import smbus
import time

#====================================================================================================
#Class to use the MPU-9150 sensor
#====================================================================================================

## MPU9150 Default I2C slave address
SLAVE_ADDRESS        = 0x68
## AK8975 I2C slave address
AK8975_SLAVE_ADDRESS = 0x0C
## Device id
DEVICE_ID            = 0x71

''' MPU-9150 Register Addresses '''
## sample rate driver
SMPLRT_DIV     = 0x19
CONFIG         = 0x1A
GYRO_CONFIG    = 0x1B
ACCEL_CONFIG   = 0x1C
FIFO_EN        = 0x23
I2C_MST_CTRL   = 0x24
I2C_MST_STATUS = 0x36
INT_PIN_CFG    = 0x37
INT_ENABLE     = 0x38
INT_STATUS     = 0x3A
ACCEL_OUT      = 0x3B
TEMP_OUT       = 0x41
GYRO_OUT       = 0x43

I2C_MST_DELAY_CTRL = 0x67
SIGNAL_PATH_RESET  = 0x68
USER_CTRL          = 0x6A
PWR_MGMT_1         = 0x6B
PWR_MGMT_2         = 0x6C
FIFO_R_W           = 0x74
WHO_AM_I           = 0x75

## Gyro Full Scale Select 250dps
GFS_250  = 0x00
## Gyro Full Scale Select 500dps
GFS_500  = 0x01
## Gyro Full Scale Select 1000dps
GFS_1000 = 0x02
## Gyro Full Scale Select 2000dps
GFS_2000 = 0x03
## Accel Full Scale Select 2G
AFS_2G   = 0x00
## Accel Full Scale Select 4G
AFS_4G   = 0x01
## Accel Full Scale Select 8G
AFS_8G   = 0x02
## Accel Full Scale Select 16G
AFS_16G  = 0x03

# AK8975 Register Addresses
AK8975_ST1        = 0x02
AK8975_MAGNET_OUT = 0x03
AK8975_CNTL       = 0x0A
AK8975_ASAX       = 0x10

# CNTL Mode select
## Power down mode
AK8975_MODE_DOWN   = 0x00
## One shot data output
AK8975_MODE_ONE    = 0x01
## Self test mode
AK8975_MODE_SELF   = 0x08
## Fuse ROM access mode
AK8975_MODE_FUSE   = 0x0F

## smbus
bus = smbus.SMBus(1)

## MPU9150 I2C Controll class
class MPU9150:

    ## Constructor
    #  @param [in] address MPU-9150 I2C slave address default:0x68
    def __init__(self, address=SLAVE_ADDRESS):
        self.address = address
        self.configMPU9150(GFS_2000, AFS_4G)
        self.configAK8975(AK8975_MODE_ONE)
        #Read offsets
        #Accelerometer X Y Z
        #Magnetometer X Y Z
        self.gyroxoffset = 0
        self.gyroyoffset = 0
        self.gyrozoffset = 0
        self.magxoffset = 0
        self.magyoffset = 0
        self.magzoffset = 0
        try:
            f = open('MPU9150offsets.txt', 'r')
            line = "\n"
            
            while line == "\n" or line[0] == "#":
                line = f.readline()
            data = line.split()
            self.gyroxoffset = float(data[0])
            self.gyroyoffset = float(data[1])
            self.gyrozoffset = float(data[2])
            print("gyroxoffset=%d gyroyoffset=%d gyrozoffset=%d" % (self.gyroxoffset, self.gyroyoffset, self.gyrozoffset))
            line="\n"
            while line == "\n" or line[0] == "#":
                line = f.readline()
            data = line.split()
            self.magxoffset = int(data[0])
            self.magyoffset = int(data[1])
            self.magzoffset = int(data[2])
            print("magxoffset=%d magyoffset=%d magzoffset=%d" % (self.magxoffset, self.magyoffset, self.magzoffset))
        except:
            print('No MPU9150offsets file found: offsets set to zero')

    ## Search Device
    #  @param [in] self The object pointer.
    #  @retval true device connected
    #  @retval false device error
    def searchDevice(self):
        who_am_i = bus.read_byte_data(self.address, WHO_AM_I)
        if(who_am_i == DEVICE_ID):
            return true
        else:
            return false

    ## Configure MPU-9150
    #  @param [in] self The object pointer.
    #  @param [in] gfs Gyro Full Scale Select(default:GFS_250[+250dps])
    #  @param [in] afs Accel Full Scale Select(default:AFS_2G[2g])
    def configMPU9150(self, gfs, afs):
        if gfs == GFS_250:
            self.gres = 250.0/32768.0
        elif gfs == GFS_500:
            self.gres = 500.0/32768.0
        elif gfs == GFS_1000:
            self.gres = 1000.0/32768.0
        else:  # gfs == GFS_2000
            self.gres = 2000.0/32768.0

        if afs == AFS_2G:
            self.ares = 2.0/32768.0
        elif afs == AFS_4G:
            self.ares = 4.0/32768.0
        elif afs == AFS_8G:
            self.ares = 8.0/32768.0
        else: # afs == AFS_16G:
            self.ares = 16.0/32768.0

        # sleep off
        bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        # x axis gyroscope reference
        bus.write_byte_data(self.address, PWR_MGMT_1, 0x01)
        time.sleep(0.1)
        # DLPF_CFG - 44Hz 
        bus.write_byte_data(self.address, CONFIG, 0x03)
        # sample rate divider - GOR/(1+SMPLRT)
        bus.write_byte_data(self.address, SMPLRT_DIV, 0x01)
        # gyro full scale select
        bus.write_byte_data(self.address, GYRO_CONFIG, gfs << 3)
        # accel full scale select
        bus.write_byte_data(self.address, ACCEL_CONFIG, afs << 3)
        # BYPASS_EN
        bus.write_byte_data(self.address, INT_PIN_CFG, 0x02)
        time.sleep(0.1)

    ## Configure AK8975
    #  @param [in] self The object pointer.
    #  @param [in] mode Magneto Mode Select(default:AK8975_MODE_C8HZ[Continous 8Hz])
    def configAK8975(self, mode):
        self.mres = 4912.0/32760.0
        
        #Power down mode
        bus.write_byte_data(AK8975_SLAVE_ADDRESS, AK8975_CNTL, 0x00)
        time.sleep(0.01)

        # set read FuseROM mode
        bus.write_byte_data(AK8975_SLAVE_ADDRESS, AK8975_CNTL, 0x0F)
        time.sleep(0.01)

        # read coef data
        data = bus.read_i2c_block_data(AK8975_SLAVE_ADDRESS, AK8975_ASAX, 3)

        self.magXcoef = (data[0] - 128) / 256.0 + 1.0
        self.magYcoef = (data[1] - 128) / 256.0 + 1.0
        self.magZcoef = (data[2] - 128) / 256.0 + 1.0

        # set power down mode
        bus.write_byte_data(AK8975_SLAVE_ADDRESS, AK8975_CNTL, 0x00)
        time.sleep(0.01)

        # set single measurement mode
        bus.write_byte_data(AK8975_SLAVE_ADDRESS, AK8975_CNTL, AK8975_MODE_ONE)
        time.sleep(0.01)

    ## brief Check data ready
    #  @param [in] self The object pointer.
    #  @retval true data is ready
    #  @retval false data is not ready
    def checkDataReady(self):
        drdy = bus.read_byte_data(self.address, INT_STATUS)
        if drdy & 0x01:
            return True
        else:
            return False

    ## Read accelerometer
    #  @param [in] self The object pointer.
    #  @retval x : x-axis data
    #  @retval y : y-axis data
    #  @retval z : z-axis data
    def readAccel(self):
        data = bus.read_i2c_block_data(self.address, ACCEL_OUT, 6)
        x = self.dataConv(data[1], data[0])
        y = self.dataConv(data[3], data[2])
        z = self.dataConv(data[5], data[4])

        x = round(x*self.ares, 3)
        y = round(y*self.ares, 3)
        z = round(z*self.ares, 3)

        return {"x":x, "y":y, "z":z}

    ## Read gyro
    #  @param [in] self The object pointer.
    #  @retval x : x-gyro data
    #  @retval y : y-gyro data
    #  @retval z : z-gyro data
    def readGyro(self):
        data = bus.read_i2c_block_data(self.address, GYRO_OUT, 6)

        x = self.dataConv(data[1], data[0])
        y = self.dataConv(data[3], data[2])
        z = self.dataConv(data[5], data[4])

        x = round(x*self.gres, 3)
        y = round(y*self.gres, 3)
        z = round(z*self.gres, 3)
        
        x = x - self.gyroxoffset
        y = y - self.gyroyoffset
        z = z - self.gyrozoffset

        return {"x":x, "y":y, "z":z}

    ## Read magneto
    #  @param [in] self The object pointer.
    #  @retval x : X-magneto data
    #  @retval y : y-magneto data
    #  @retval z : Z-magneto data
    def readMagnet(self):
        x=0
        y=0
        z=0
        
        #Set single measurement mode
        bus.write_byte_data(AK8975_SLAVE_ADDRESS, AK8975_CNTL, AK8975_MODE_ONE)
        time.sleep(0.01)
        
        # Wait for dataready
		drdy = 0
		while (drdy & 0x01) == 0:
		    drdy = bus.read_byte_data(AK8975_SLAVE_ADDRESS, AK8975_ST1)
	        
	        # check data ready
	        if drdy & 0x01 :
	            data = bus.read_i2c_block_data(AK8975_SLAVE_ADDRESS, AK8975_MAGNET_OUT, 7)

	            # check overflow
	            if (data[6] & 0x08)!=0x08:
	                x = self.dataConvM(data[0], data[1])
	                y = self.dataConvM(data[2], data[3])
	                z = self.dataConvM(data[4], data[5])

	                x = round(x * self.mres * self.magXcoef, 3)
	                y = round(y * self.mres * self.magYcoef, 3)
	                z = round(z * self.mres * self.magZcoef, 3)
	                
	                x = x - self.magxoffset
	                y = y - self.magyoffset
	                z = z - self.magzoffset

        return {"x":x, "y":y, "z":z}

    ## Read temperature
    #  @param [out] temperature temperature(degrees C)
    def readTemperature(self):
        data = bus.read_i2c_block_data(self.address, TEMP_OUT, 2)
        temp = self.dataConv(data[1], data[0])

        temp = round((temp / 333.87 + 21.0), 3)
        return temp

    ## Data Convert
    # @param [in] self The object pointer.
    # @param [in] data1 LSB
    # @param [in] data2 MSB
    # @retval Value MSB+LSB(int 16bit)
    def dataConv(self, data1, data2):
        value = data1 | (data2 << 8)
        if(value & (1 << 16 - 1)):
            value -= (1<<16)
        return value
    
    ## Data Convert for Magnetometer
    # @param [in] self The object pointer.
    # @param [in] data1 LSB
    # @param [in] data2 MSB
    # @retval Value MSB+LSB(int 16bit)
    def dataConvM(self, data1, data2):
        value = data1 | (data2 << 8)
        if(value > 0x0FFF):
            value = value - 0xF000
            value = 0x0FFF - value + 1
            value = -int(value)
        else:
            value = int(value)
        return value
