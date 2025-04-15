
from lola import Robot
import time
import signal

class Nao:
    def __init__(self):
        #super().__init__()
        
        # copied from JointData.h
        # order of joints - this is the same order as in the naoth
        #    but not the same as lola
        self.joint_names = [
        
          # head
          "HeadPitch",
          "HeadYaw",

          # arms
          "RShoulderRoll",
          "LShoulderRoll",
          "RShoulderPitch",
          "LShoulderPitch",

          "RElbowRoll",
          "LElbowRoll",
          "RElbowYaw",
          "LElbowYaw",

          # legs
          "RHipYawPitch", # does not exist on the robot
          "LHipYawPitch",
          "RHipPitch",
          "LHipPitch",
          "RHipRoll",
          "LHipRoll",
          "RKneePitch",
          "LKneePitch",
          "RAnklePitch",
          "LAnklePitch",
          "RAnkleRoll",
          "LAnkleRoll",
          
          # NOTE: those values don't exist on the old V3.2/V3.3 robots
          #       so, we put them at the end for easier support for the old format
          "LWristYaw",
          "RWristYaw",
          "LHand",
          "RHand"
        ]
        
        self.jointMotorData     = { name : 0.0 for name in self.joint_names }
        self.jointStiffnessData = { name : 1.0 for name in self.joint_names }
        self.jointSensorData    = { name : 0.0 for name in self.joint_names }
        
        self.accelerometerData  = [0.0, 0.0, 0.0]
        self.gyroData           = [0.0, 0.0, 0.0]
        self.imuData            = [0.0, 0.0, 0.0]
		
        # connect to lola
        self.robot = Robot()
        
        self.initialized = False
        self.running = True
        
        # initial read
        data = self.robot.read()
        positions_value = data[ "Position" ]
        for index,name in enumerate (self.robot.joints):
            self.jointSensorData[name] = positions_value[index]
            
        signal.signal(signal.SIGINT, self.exit_handler)

    def setStiffness(self, value):
        self.jointStiffnessData = { name : value for name in self.joint_names }
    
    def getTime(self):
        return time.time()
        
    def update(self):
        
        if not self.initialized:
            self.initialized = True
            return True
        
        # ovverride stiffness
        running = self.running # copy is important!
        if not running:
            print("set stiffness to 0")
            for name in self.joint_names:
                self.jointStiffnessData[name] = 0.0
        
        
        # pack the joint data and stiffness
        for name in self.joint_names:
            if name != "RHipYawPitch":
                self.robot.command( "Position", name ,self.jointMotorData[name])
                
                stiffness = max(min(self.jointStiffnessData[name], 1.0), 0.0) 
                self.robot.command( "Stiffness", name ,stiffness)
        
        self.robot.send()
        
        # read all joint sensors
        data = self.robot.read() #['lolaSensors']
        
        #print(data)
        
        # unpack position sensors values
        positions_value = data[ "Position" ]
        for index,name in enumerate (self.robot.joints):
            self.jointSensorData[name] = positions_value[index]
            
        self.accelerometerData = data[ "Accelerometer" ]
        self.gyroData          = data[ "Gyroscope" ]
        self.imuData           = data[ "Angles" ] + [0.0]
        
        if not running:
            self.robot.close()
            return False  
        else:
            return True
            
    def stop(self):
        self.running = False
        self.update()
        
    def exit_handler(self, signum, frame):
        self.running = False
        
        
