
from Nao import Nao

nao = Nao()

for name in nao.joint_names:
    nao.jointStiffnessData[name] = 0.0

t_0 = nao.getTime()

while nao.update():
    
    t = nao.getTime() - t_0
    
    yaw = nao.jointSensorData["HeadYaw"]
    
    print("{:.5}: {}".format(t,yaw))

    #for name in nao.joint_names:
    #    nao.jointMotorData[name] = nao.jointSensorData[name]
    #    nao.jointStiffnessData[name] = stiffness



