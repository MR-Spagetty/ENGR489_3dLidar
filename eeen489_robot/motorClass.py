from roboclaw_3 import Roboclaw
import sys
import math
 
class MotorClass:
    def __init__(self,device: str = '/dev/ttyAMA0', baud: int = 38400, timeout: int = 0, retries: int = 1,l_x: float = 0.1680, l_y: float = 0.2075, wheelDiameter: float = 0.096, PPR: float = 537.7):
        """
        : param device: The serial bus address of the Roboclaw motor drivers.
        : type device: String
        : param baud: The baud rate of the serial bus described in device.
        : type baud: Integer
        """
        self.device = device
        self.baud = baud
        self.timeout = timeout
        self.retries = retries
        self.rc = Roboclaw(self.device, self.baud, self.timeout, self.retries)
        self.openStatus = self.rc.Open()
        if self.openStatus == 0:
            print("Could not open serial bus on device %s",self.device)
            print("Check that the serial device is %s and the communication baud rate is %d",self.device,self.baud)
        # else:
            # Ensure we can read the roboclaws at teh motor addressses provided ,otherwise don't move motors
            # for motor in self.motors:
                # obtain version if version is rubbish set openStatus = 0 and break
                # sys.exit(1)
        self.lengthX = l_x
        self.lengthY = l_y
        self.wheelDiameter = wheelDiameter
        self.PPR = PPR # Don't want the students to modify this
    def forwardKinematics(self,Vx: float,Vy: float ,Wz: float):
        

        """
        : param Vx: The linear velocity that the robot should travel in the x-direction.
        : type Vx: Float
        : param Vy: The linear velocity that the robot should travel in the y-direction.
        : type Vy: Float
        : param Wz: The angular velocity that the robot should rotate about the z-axis.
        : type Wz: Float
        : returns: The rotational velocity of motors w1, w2, w3, and w4.
        : rtype: Integer
        """
        w1 = (Vx - Vy -(self.lengthX + self.lengthY)*Wz)/(self.wheelDiameter/2)
        w2 = (Vx + Vy +(self.lengthX + self.lengthY)*Wz)/(self.wheelDiameter/2)
        w3 = (Vx + Vy -(self.lengthX + self.lengthY)*Wz)/(self.wheelDiameter/2)
        w4 = (Vx - Vy +(self.lengthX + self.lengthY)*Wz)/(self.wheelDiameter/2)
        return w1,w2,w3,w4
    def angularToSpeed(self,w1,w2,w3,w4):
        """
        : param w1: The angular velocity of mecanum wheel one (rad.s^-1)
        : type w1: float

        : param w2: The angular velocity of mecanum wheel two (rad.s^-1)
        : type w2: float

        : param w3: The angular velocity of mecanum wheel three (rad.s^-1)
        : type w3: float

        : param w4: The angular velocity of mecanum wheel four (rad.s^-1)
        : type w4: float

        : returns: The speed to input into the motor driver in quadrature pulses per second (QPPS).
        : rtype: Tuple (float, float, float, float)
        """
        s1 = w1 * self.PPR / (2 * math.pi)
        s2 = w2 * self.PPR / (2 * math.pi)
        s3 = w3 * self.PPR / (2 * math.pi)
        s4 = w4 * self.PPR / (2 * math.pi)
        return s1,s2,s3,s4
    def angularToSpeedTest(self,w):
        """
        : param w: Tuple of the angular velocities of each mecanum wheel (rad.s^-1)
        : type w1: Tuple (float, float, float, float)

        : returns: Tuple of the speeds to input into the motor driver in quadrature pulses per second (QPPS).
        : rtype: Tuple (float, float, float, float)
        """
        s = tuple(w)
        for i in range(len(w)):
            s[i] = w[i] * self.PPR / (2*math.pi)
        return s    
    def movement(self,s1,s2,s3,s4):

        """
        : param s1: The Roboclaw speed to drive mecanum wheel one (QPPS)
        : type s1: float

        : param s2: The Roboclaw speed to drive mecanum wheel two (QPPS)
        : type s2: float

        : param s3: The Roboclaw speed to drive mecanum wheel three (QPPS)
        : type s3: float

        : param s4: The Roboclaw speed to drive mecanum wheel four (QPPS)
        : type s4: float

        : returns: The error state of the Roboclaw motor driver 0 = Error detected 1 = No error detected.
        : rtype: Integer
        """
        if self.openStatus:
            if not self.rc.SpeedM1M2(0x80,int(s1),int(s3)):
                print("Command error or timeout when setting motor one speed to %d and motor two speed to %d on Roboclaw at address %s"%(int(s1),int(s3),"0x{:02x}".format(0x80)))
                return 0
            if not self.rc.SpeedM1M2(0x81,int(s4),int(s2)):
                print("Command error or timeout when setting motor one speed to %d and motor two speed to %d on Roboclaw at address %s"%(int(s2),int(s4),"0x{:02x}".format(0x81)))
                return 0
            return 1
        else:
            return 0
    def motorgetspeed():
        return

    def inverseKinematics(self, w1, w2, w3, w4):
        """
        : param w1: The rotational velocity that mecanum wheel one is travelling at (rad.s^-1).
        : type w1: float

        : param w2: The rotational velocity that mecanum wheel two is travelling at (rad.s^-1).
        : type w2: float

        : param w3: The rotational velocity that mecanum wheel three is travelling at (rad.s^-1).
        : type w3: float

        : param w4: The rotational velocity that mecanum wheel four is travelling at (rad.s^-1).
        : type w4: float

        : returns: The linear velocity of the robot platform in the x and y directions, and the rotational velocity about the x-axis.
        : rtype: Tuple(Float, Float, Float)
        """
        Vx = (w1+w2+w3+w4)*(self.wheelDiameter/2)/4 # all should be the same
        Vy = (-w1+w2+w3-w4)*(self.wheelDiameter/2)/4 #1-3 op , 2-4, op
        Wz = (-w1 + w2 - w3 + w4)*(self.wheelDiameter/2) / (4*(self.lengthX + self.lengthY)) #1-2, 3-4 opposite
        # return the Longitudinal, Translational and Angular velocity of the motors
        return Vx, Vy, Wz

    def speedToAngular(self,s1,s2,s3,s4):
        """
        : param s1: The Roboclaw speed that mecanum wheel one is travelling at (QPPS)
        : type s1: float

        : param s2: The Roboclaw speed that mecanum wheel two is travelling at (QPPS)
        : type s2: float

        : param s3: The Roboclaw speed that mecanum wheel three is travelling at (QPPS)
        : type s3: float

        : param s4: The Roboclaw speed that mecanum wheel four is travelling at (QPPS)
        : type s4: float

        : returns: The rotational velocity that each mecanum wheel is moving (rad.s^-1).
        : rtype: Tuple (float, float, float, float)
        """
        # Converts from Quadrature Encoder Pulses Per Second to Angular Velocity rad.s^1
        # Takes in a 4 element NumPY vector and returns a 4 element NumPy vector.
        w1 = s1 * 2 * math.pi / self.PPR
        w2 = s2 * 2 * math.pi / self.PPR
        w3 = s3  * 2 * math.pi / self.PPR
        w4 = s4 * 2 * math.pi / self.PPR
        return w1,w2,w3,w4
    def speedToAngularTest(self,s):
        """
        : param s: Tuple of the Roboclaw speeds that the mecanum wheels are travelling at (QPPS)
        : type s: Tuple(float, float, float, float)

        : returns: The rotational velocities that the mecanum wheels are moving (rad.s^-1).
        : rtype: Tuple (float, float, float, float)
        """
        w = tuple(s)
        for i in range(len(s)):
            w[i] = s[i] * (2*math.pi) / self.PPR
        return w 

    def encoders(self):
        """
        : returns: A 5 element tupple of integers
        :       Element Zero is the error state 0 = timeout or CRC failure 1 = successful encoder read
        :       Element One is the speed of the encoder attached to motor one (QPPS)
        :       Element Two is the speed of the encoder attached to motor two (QPPS)
        :       Element Three is the speed of the encoder attached to motor three (QPPS)
        :       Element Four is the speed of the encoder attached to motor four (QPPS)
        :   rtype: Tuple(Integer, Integer, Integer, Integer, Integer)
        """
        if self.openStatus:
            encoderVelocity1 = self.rc.ReadSpeedM1(0x80)
            encoderVelocity4 = self.rc.ReadSpeedM1(0x81)
            encoderVelocity3 = self.rc.ReadSpeedM2(0x80)
            encoderVelocity2 = self.rc.ReadSpeedM2(0x81)
            if(encoderVelocity1[0] & encoderVelocity2[0] & encoderVelocity3[0] & encoderVelocity4[0]):
                # The encoder read was successful
                return(1,encoderVelocity1[1],encoderVelocity2[1],encoderVelocity3[1],encoderVelocity4[1])
            else:
                # Did not obtain encoder values timeout
                print("Returned value (0,0): timeout or CRC check failure.")
                # return zero or trigger exception - might be better to trigger an exception so that the robot does not think it is stopped.
                return(0,0,0,0,0)
        else:
            # motors not open return error
            return(0,0,0,0,0)

    def getVelocity(self):
        """
        :   returns: A 4 element tuple of integers. 
        :       Element Zero is the error state 0 = timeout or CRC failure 1 = successful encoder read
        :       Element One is the linear velocity of the robot in the x-direction.
        :       Element Two is the linear velocity of the robot in the y-direction.
        :       Element Three is the rotational velocity of the robot about the z-axis.
        :   rtype: Tuple(integer, float, float, float)
        """
        encoderSpeed = self.encoders()
        if (encoderSpeed[0] == 0):
            # didn't read the encoders, don't report anything back
            return(0,0.0,0.0,0.0)
        [encoderVelocity1,encoderVelocity2,encoderVelocity3,encoderVelocity4] = self.speedToAngular(encoderSpeed[1],encoderSpeed[2],encoderSpeed[3],encoderSpeed[4])
        [Vx, Vy, Wz] = self.inverseKinematics(encoderVelocity1,encoderVelocity2,encoderVelocity3,encoderVelocity4)
        return(1,Vx,Vy,Wz)
    
    def setVelocity(self, Vx: float = 0.0, Vy: float = 0.0, Wz: float = 0.0):
        """
        : param Vx: The linear velocity that the robot should travel in the x-direction (m.s^-1).
        : type Vx: Float

        : param Vy: The linear velocity that the robot should travel in the y-direction (m.s^-1).
        : type Vy: Float

        : param Wz: The angular velocity that the robot should travel about the z-axis (rad.s^-1).
        : type Wz: Float

        : returns: Boolean value based on the outcome 1 = Motors moved (no error) 0 = Motor timeout or CRC fail (error)
        : rtype: Integer
        """
        # check velocity value in range? max ~1.5 m.s^-1
        [rotationalVelocity1,rotationalVelocity2,rotationalVelocity3,rotationalVelocity4] = self.forwardKinematics(Vx,Vy,Wz)
        [speed1,speed2,speed3,speed4] = self.angularToSpeed(rotationalVelocity1,rotationalVelocity2,rotationalVelocity3,rotationalVelocity4)
        if self.movement(speed1,speed2,speed3,speed4):
            # Set the robot velocity successfully.
            return 1
        # if something went wrong turn off all of the motors    
        self.movement(0,0,0,0)
        return 0