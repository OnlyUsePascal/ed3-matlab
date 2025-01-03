from math import cos, pi, radians, sin
# import numpy as np

time_history = [0]
x_history = [0]
y_history = [0]
phi_history = [0]

v1_history = [0]
v2_history = [0]
v3_history = [0]
v4_history = [0]
vCarX_history = [0]
vCarY_history = [0]
vCarPhi_history = [0]

phi1_history = [0]
phi2_history = [0]
phi3_history = [0]
phi4_history = [0]

l = 0.5; # Distance between wheel pairs (m)
d = 0.5; # Distance between wheels along the axis (m)
wheelR = 0.097; 

# Inverse kinematics matrix 
J_inv = [[1, -1, -(l + d)],
         [1,  1, -(l + d)],
         [1, -1,  (l + d)],
         [1,  1,  (l + d)]]

def start_move(distance, angleMove, angleSpin, time, step):
  global time_history

  dt = time / step
  time_array = [i * dt for i in range(step)]
  # print(time_array)
  time_history_last = time_history[-1]
  time_history += ([round(i + time_history_last, 2) for i in time_array[1:]])
  
  angleMove = radians(angleMove)
  angleSpin = radians(angleSpin)
  vCarX = distance * cos(angleMove) / time
  vCarY = distance * sin(angleMove) / time
  vCarPhi = angleSpin / time
  
  # wheelAngularVelocities = [vCarX, vCarY, vCarPhi] * J_inv / wheelR
  v1, v2, v3, v4 = (1 / wheelR) * (vCarX - vCarY - (l + d) * vCarPhi), \
                    (1 / wheelR) * (vCarX + vCarY - (l + d) * vCarPhi), \
                    (1 / wheelR) * (vCarX - vCarY + (l + d) * vCarPhi), \
                    (1 / wheelR) * (vCarX + vCarY + (l + d) * vCarPhi)

  print(f'vcarX: {vCarX}, vcarY: {vCarY}, vcarPhi: {vCarPhi}') 
  for i in range(step):
    dPhi = vCarPhi * dt
    phi = phi_history[-1] + dPhi
    x_pos, y_pos = x_history[-1] + (vCarX * cos(phi) - vCarY * sin(phi)) * dt, \
                  y_history[-1] + (vCarX * sin(phi) + vCarY * cos(phi)) * dt
    
    phi1_pos, phi2_pos, phi3_pos, phi4_pos = phi1_history[-1] + v1 * dt, \
                                              phi2_history[-1] + v2 * dt, \
                                              phi3_history[-1] + v3 * dt, \
                                              phi4_history[-1] + v4 * dt
    
    v1_history.append(v1)
    v2_history.append(v2)
    v3_history.append(v3)
    v4_history.append(v4)
    phi1_history.append(phi1_pos)
    phi2_history.append(phi2_pos)
    phi3_history.append(phi3_pos)
    phi4_history.append(phi4_pos)
    
    vCarX_history.append(vCarX)
    vCarY_history.append(vCarY)
    vCarPhi_history.append(vCarPhi)
    x_history.append(x_pos)
    y_history.append(y_pos)
    phi_history.append(phi)
    
def square_with_turn(distance, timePerOp, stepPerOp):
  start_move(distance, 0, 0, timePerOp, stepPerOp)
  # start_move(0, 0, 90, timePerOp, stepPerOp)
  
  # for i in range(4):
  #   start_move(distance, 0, 0, timePerOp, stepPerOp)
  #   start_move(0, 0, 90, timePerOp, stepPerOp)


square_with_turn(5, 1, 4)
# function square_with_turn(distance, timePerOp, stepPerOp) 
#     % global timePerOp stepPerOp;

#     for i = 1:4
#         % Move straight
#         move(distance, 0, 0, timePerOp, stepPerOp);

#         % Turn 90°
#         move(0, 0, 90, timePerOp, stepPerOp);
#     end
  
# print(time_history)
# print(vCarX_history, '\n', vCarY_history, '\n', vCarPhi_history)
# print(x_history, '\n', y_history, '\n', phi_history)

# phi1_history = phi1_history / (2 * pi)
# phi2_history = phi2_history / (2 * pi)
# phi3_history = phi3_history / (2 * pi)
# phi4_history = phi4_history / (2 * pi)

phi1_history = [i / (2 * pi) for i in phi1_history]
phi2_history = [i / (2 * pi) for i in phi2_history]
phi3_history = [i / (2 * pi) for i in phi3_history]
phi4_history = [i / (2 * pi) for i in phi4_history]

print('> velocities')
print(v1_history, '\n', v2_history, '\n', v3_history, '\n', v4_history)

print('> positions')
print(phi1_history, '\n', phi2_history, '\n', phi3_history, '\n', phi4_history)
  