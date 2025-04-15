from controller import Robot, TouchSensor, Camera, Motor, PositionSensor
from enum import Enum

# combine a list of motors and controll them as it is a single motor
class MultiMotor:
    def __init__(self, motors):
        self.motors = motors
    
    def __getattr__(self, name):
        # apply any method call to both motors
        def method(*args, **kwargs):
            results = []
            # apply methods to all motors
            for m in self.motors:
                results += [getattr(m, name)(*args, **kwargs)]
            
            return results
        return method

class Nao(Robot):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())

        # cameras
        self.camera_top    = self.getDevice("CameraTop")
        self.camera_bottom = self.getDevice("CameraBottom")
        
        # orientation sensors
        self.accelerometer_sensor = self.getDevice("accelerometer")
        self.gyro_sensor          = self.getDevice("gyro")
        self.imu_sensor           = self.getDevice("inertial unit")

        # enable orientation sensors
        self.accelerometer_sensor.enable(self.timestep)
        self.gyro_sensor.enable(self.timestep)
        self.imu_sensor.enable(self.timestep)


        # TODO add fsr like in webots nao demo
        self.fsr_r = self.getDevice("RFsr")
        self.fsr_r.enable(self.timestep)
        # TODO move enable down to the other enable statements when the correct fsr implementation is finished
        self.fsr_l = self.getDevice("LFsr")
        self.fsr_l.enable(self.timestep)
        
        
        # Individual joints of the fingers. 
        # On the NAO they cannot be controlled individually.
        # Thus, we don't use them directly.
        
        # hands: bind individual joints of the fingers to behave like a single motor like on the real NAO
        self.lHand = MultiMotor([self.getDevice(name) for name in [
            "LPhalanx1", "LPhalanx2", "LPhalanx3", "LPhalanx4", "LPhalanx5", "LPhalanx6", "LPhalanx7", "LPhalanx8"
        ]])
        
        self.rHand = MultiMotor([self.getDevice(name) for name in [
            "RPhalanx1", "RPhalanx2", "RPhalanx3", "RPhalanx4", "RPhalanx5", "RPhalanx6", "RPhalanx7", "RPhalanx8"
        ]])
        
        class MyEnum(str, Enum):
            state1 = 'state1'
            state2 = 'state2'
        
        # create actuator members
        # order of joints - needed for webots
        self.joint_names = [
          "HeadYaw",
          "HeadPitch",
          "LShoulderPitch",
          "LShoulderRoll",
          "LElbowYaw",
          "LElbowRoll",
          "LWristYaw",
          "LHipYawPitch",
          "RHipYawPitch",#nicht in lola
          "LHipRoll",
          "LHipPitch",
          "LKneePitch",
          "LAnklePitch",
          "LAnkleRoll",
          "RHipRoll",
          "RHipPitch",
          "RKneePitch",
          "RAnklePitch",
          "RAnkleRoll",
          "RShoulderPitch",
          "RShoulderRoll",
          "RElbowYaw",
          "RElbowRoll",
          "RWristYaw",
          "LHand",
          "RHand"
        ]
        """
        self.joint_names = [
            "HeadYaw", #1
            "HeadPitch", #2
            "RShoulderPitch", #3
            "LShoulderPitch", #4 
            "RShoulderRoll", #5
            "LShoulderRoll", #6
            "RElbowYaw", #7 
            "LElbowYaw", #8 
            "RElbowRoll", #9
            "LElbowRoll", #1ß
            "RHipYawPitch", #11
            "RHipPitch", #12
            "RHipRoll", #13
            "RKneePitch", #14
            "RAnklePitch", #15
            "RAnkleRoll", #16
            "LHipYawPitch", #17
            "LHipPitch", #18
            "LHipRoll", #19
            "LKneePitch", #20
            "LAnklePitch", #21
            "LAnkleRoll", #22
            "RWristYaw", #23
            "LWristYaw" #24
        ]"""
        
        self.jointMotorData     = { name : 0 for name in self.joint_names }
        self.jointStiffnessData = { name : 1 for name in self.joint_names }
        self.jointSensorData    = { name : 0 for name in self.joint_names }
        
        # list of joint actuators and sensors
        self.joint_motors   = { name : self.getDevice(name)       for name in self.joint_names }
        self.joint_sensors = { name : self.getDevice(name + "S") for name in self.joint_names }
        
        
        # enable joint position sensors
        for joint in self.joint_sensors.values():
          if joint is not None:
            joint.enable(self.timestep)
            
        # experimental: set more realistic PID parameters
        for j in self.joint_motors.values():
          if j is not None:
            j.setControlPID(40,1,0.3)
        
        self.accelerometerData = [0.0, 0.0, 0.0]
        self.gyroData = [0.0, 0.0, 0.0]
        self.imuData = [0.0, 0.0, 0.0]
        
    #def get_joint_positions(self):
    #    return [ self.joint_sensors[name].getValue() for name in self.joint_names ]

    def update(self):
        
        # write all joints
        for name in self.joint_names:
          joint = self.joint_motors[name]
          if joint is not None:
            stiffness = max(min(self.jointStiffnessData[name], 1), 0) 
            joint.setAvailableTorque( 0.2 + 0.8*stiffness * joint.getMaxTorque() )
            joint.setPosition( self.jointMotorData[name] )
        
        # execute simulation step
        result = self.step(self.timestep)
        
        # read all joint sensors
        for name in self.joint_names:
          joint = self.joint_sensors[name]
          if joint is not None:
            self.jointSensorData[name] = joint.getValue()
        
        # read orientation values
        self.accelerometerData = self.accelerometer_sensor.getValues()
        self.imuData = self.imu_sensor.getRollPitchYaw()
        self.gyroData = self.gyro_sensor.getValues()

        return( result != -1 )
