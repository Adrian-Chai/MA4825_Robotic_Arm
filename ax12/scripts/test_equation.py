import math
import re
import numpy as np
coordinate = np.transpose(np.array([[0,10,2]]))

LINK_1 = 10 #9.36
LINK_2 =  16.1 #16.07

x = coordinate[0][0]
y = coordinate[1][0]
z = coordinate[2][0]

yaw = math.atan(y/(x-1.5))
if yaw < 0:
    yaw = yaw + math.pi

H01 = np.array([[math.cos(yaw),-math.sin(yaw),0,1.5],
                [math.sin(yaw),math.cos(yaw) ,0,0  ],
                [0            ,0             ,1,7.5],
                [0            ,0             ,0,1  ]
                 ])

motor1_point = H01 @ np.transpose(np.array([[1.5,0,2,1]]))

relative_distance = math.sqrt((x-motor1_point[0][0])**2+(y-motor1_point[1][0])**2) 

relative_height = z - motor1_point[2] 

position0 = 150 + math.degrees(yaw)

theta2 = -math.acos((relative_distance**2 + relative_height**2 - LINK_1**2 - LINK_2**2)/(2*LINK_1*LINK_2))
theta1 = math.atan(relative_height/relative_distance) - math.atan((LINK_2*math.sin(theta2)/(LINK_1+LINK_2*math.cos(theta2))))
theta1 = math.pi + theta1
print(math.degrees(theta1))
print(math.degrees(theta2))


position1 = 120 + (90 -math.degrees(theta1))
position2 = 225 + math.degrees(theta2) 

print(position0)
print(position1)
print(position2)
