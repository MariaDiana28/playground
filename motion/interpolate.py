import numpy as np
from scipy.interpolate import CubicSpline
import math

from Nao import Nao

nao = Nao()

for name in nao.joint_names:
    nao.jointStiffnessData[name] = 0.0

nao.jointStiffnessData["HeadYaw"] = 0.5

t_0 = nao.getTime()

target_yaw = 0.6
currect_yaw = nao.jointSensorData["HeadYaw"]
time = 2
spline = CubicSpline([0, time], [currect_yaw, target_yaw], bc_type='clamped')


xt = currect_yaw

while nao.update():
    
    t = nao.getTime() - t_0
    yaw = nao.jointSensorData["HeadYaw"]
    print("{:.3}: {}".format(t,yaw))
    
    if t >= time:
        break
    
    xt += np.clip(target_yaw - xt, -0.01, 0.01)
    print(spline(t))
    nao.jointMotorData["HeadYaw"] = spline(t).item()#xt


nao.stop()

