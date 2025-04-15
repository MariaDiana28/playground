
import math
from Nao import Nao

nao = Nao()

for name in nao.joint_names:
    nao.jointStiffnessData[name] = 1.0

nao.jointStiffnessData["HeadYaw"] = 0.5

t_0 = nao.getTime()

while nao.update():
    
    t = nao.getTime() - t_0
    yaw = nao.jointSensorData["HeadYaw"]
    print("{:.3}: {}".format(t,yaw))

    nao.jointMotorData["HeadYaw"] = math.sin(t)


