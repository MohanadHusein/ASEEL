import pybullet as p
import time
import pybullet_data
import numpy as np
from scipy import interpolate
import math
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,.5]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])

robot = p.loadURDF("\\Desktop\\ASEEL\\urdf\\ASEEL.urdf",cubeStartPos, cubeStartOrientation) # path to URDF file

#
# slide0 = p.addUserDebugParameter("base_FR_shoulder_joint", -120, 120, 1)
# slide1 = p.addUserDebugParameter("shoulder_FR_hip_joint", -120, 120, 1)
# slide2 = p.addUserDebugParameter("FR_leg_joint", 30, 150, 1)
# # #
# slide4 = p.addUserDebugParameter("base_BR_shoulder_joint", -120, 120, 1)
# slide5 = p.addUserDebugParameter("shoulder_BR_hip_joint", -120, 120, 1)
# slide6 = p.addUserDebugParameter("BR_leg_joint", 30, 150, 1)
# # # #
# slide8 = p.addUserDebugParameter("base_FL_shoulder_joint", -120, 120, 1)
# slide9 = p.addUserDebugParameter("shoulder_FL_hip_joint", -120, 120, 1)
# slide10 = p.addUserDebugParameter("FL_leg_joint", 30, 150, 1)
# # #
# slide12 = p.addUserDebugParameter("base_BL_shoulder_joint", -120, 120, 1)
# slide13 = p.addUserDebugParameter("shoulder_BL_hip_joint", -120, 120, 1)
# slide14 = p.addUserDebugParameter("BL_leg_joint", 30, 150, 1)
# #
# slide15 = p.addUserDebugParameter("x target", -0.4, 0.4, 0.001)
# slide16 = p.addUserDebugParameter("y target", -0.4, 0.4, 0.001)
# slide17 = p.addUserDebugParameter("z target", -0.4, 0.4, 0.001)

# slide18 = p.addUserDebugParameter("height", 0.2, 0.4, 0.01)
# slide18 = p.addUserDebugParameter("height", 0.2, 0.4, 0.01)

# slide0 = p.addUserDebugParameter("x target", 0, 1, 0.001)
# slide1 = p.addUserDebugParameter("y target", 0, 1, 0.001)
# slide2 = p.addUserDebugParameter("z target", 0, 1, 0.001)

def read_acc_data():
    x,y,z = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(robot)[1])
    print('X:', round(math.degrees(x),2), 'Y:', round(math.degrees(y),2), 'Z:', round(math.degrees(z),2))
    return round(math.degrees(y),2)
def calculate_x_y_z(x, y, z):
    l1 = 0.0439
    l2 = 0.15
    l3 = 0.15

    a1 = (-np.arctan2(-y,x) - np.arctan2(np.sqrt(x**2+y**2-l1**2), -l1))*-1
    D = (x**2+y**2-l1**2+z**2-l2**2-l3**2)/(2*l2*l3)
    a3 = np.arctan2(np.sqrt(1-D**2), D)
    a2 = np.arctan2(z, np.sqrt(x**2+y**2-l1**2))-np.arctan2(l3*np.sin(a3),l2+l3*np.cos(a3))

    a1 = round(np.rad2deg(a1))
    a2 = round(np.rad2deg(a2))
    a3 = round(np.rad2deg(a3))
    # print('-----------------')
    # print(a1, a2, a3)

    if a1 > 90 or a1 < -90 or a2 > 120 or a2 < -120 or a3 > 150 or a3 < 30:
        return np.nan, np.nan, np.nan
    else:
        return a1, a2, a3


ef_loc = {'FR':[None, None, None], 'FL':[None, None, None], 'BL':[None, None, None], 'BR': [None, None, None]}


def home_position():
    move_ef('FR', [-0.0439, 0.2, 0])
    move_ef('FL', [-0.0439, 0.2, 0])
    move_ef('BL', [-0.0439, 0.2, 0])
    move_ef('BR', [-0.0439, 0.2, 0])

    # move_ef('FR', [-0.022, 0.15, 0])
    # move_ef('FL', [-0.022, 0.15, 0])
    # move_ef('BL', [-0.022, 0.15, 0])
    # move_ef('BR', [-0.022, 0.15, 0])

    # move_ef('FR', [-0.0439*2, 0.20, 0])
    # move_ef('FL', [-0.0439*2, 0.20, 0])
    # move_ef('BL', [-0.0439*2, 0.20, 0])
    # move_ef('BR', [-0.0439*2, 0.20, 0])


def move_ef(leg, new_loc):
    force = 10
    a, b, c = calculate_x_y_z(new_loc[0], new_loc[1], new_loc[2])
    if not(np.isnan(a) or np.isnan(b) or np.isnan(c)):
        ef_dict = {'FR': [0, 1, 2], 'BR': [4, 5, 6], 'FL': [8, 9, 10], 'BL': [12, 13, 14]}
        if leg == 'FR':
            p.setJointMotorControl2(robot, ef_dict[leg][0], p.POSITION_CONTROL, -np.deg2rad(a), force=force)
            p.setJointMotorControl2(robot, ef_dict[leg][1], p.POSITION_CONTROL, np.deg2rad(b), force=force)
            p.setJointMotorControl2(robot, ef_dict[leg][2], p.POSITION_CONTROL, np.deg2rad(c), force=force)
        elif leg == 'FL':
            p.setJointMotorControl2(robot, ef_dict[leg][0], p.POSITION_CONTROL, -np.deg2rad(a), force=force)
            p.setJointMotorControl2(robot, ef_dict[leg][1], p.POSITION_CONTROL, -np.deg2rad(b), force=force)
            p.setJointMotorControl2(robot, ef_dict[leg][2], p.POSITION_CONTROL, np.deg2rad(c), force=force)
        elif leg == 'BL':
            p.setJointMotorControl2(robot, ef_dict[leg][0], p.POSITION_CONTROL, np.deg2rad(a), force=force)
            p.setJointMotorControl2(robot, ef_dict[leg][1], p.POSITION_CONTROL, -np.deg2rad(b), force=force)
            p.setJointMotorControl2(robot, ef_dict[leg][2], p.POSITION_CONTROL, np.deg2rad(c), force=force)
        elif leg == 'BR':
            p.setJointMotorControl2(robot, ef_dict[leg][0], p.POSITION_CONTROL, np.deg2rad(a), force=force)
            p.setJointMotorControl2(robot, ef_dict[leg][1], p.POSITION_CONTROL, np.deg2rad(b), force=force)
            p.setJointMotorControl2(robot, ef_dict[leg][2], p.POSITION_CONTROL, np.deg2rad(c), force=force)
        ef_loc[leg] = new_loc
        p.stepSimulation()
        time.sleep(1. / 240.)
    else:
        print(f'shit one of the values is nan: angle0 = {math.degrees(a)}, angle1 = {math.degrees(b)}, angle2 = {math.degrees(c)}')

def translate_torso(direction, value):
    if direction is 'up':
        for ef in ef_loc.keys():
            move_ef(ef, [ef_loc[ef][0], ef_loc[ef][1]+value, ef_loc[ef][2]])
            print([ef_loc[ef][0], ef_loc[ef][1]+value, ef_loc[ef][2]])

    if direction is 'down':
        for ef in ef_loc.keys():
            move_ef(ef, [ef_loc[ef][0], ef_loc[ef][1]-value, ef_loc[ef][2]])
            print([ef_loc[ef][0], ef_loc[ef][1]-value, ef_loc[ef][2]])

    if direction is 'fwd':
        for ef in ef_loc.keys():
            move_ef(ef, [ef_loc[ef][0], ef_loc[ef][1], ef_loc[ef][2] + value])

    if direction is 'bkwd':
        for ef in ef_loc.keys():
            move_ef(ef, [ef_loc[ef][0], ef_loc[ef][1], ef_loc[ef][2] - value])

    if direction is 'right':
        for ef in ef_loc.keys():
            if ef is 'FR' or ef is 'BR':
                move_ef(ef, [ef_loc[ef][0]-value, ef_loc[ef][1]-value/2, ef_loc[ef][2]])
            if ef is 'FL' or ef is 'BL':
                move_ef(ef, [ef_loc[ef][0]+value, ef_loc[ef][1]+value/2, ef_loc[ef][2]])

    if direction is 'left':
        for ef in ef_loc.keys():
            if ef is 'FR' or ef is 'BR':
                move_ef(ef, [ef_loc[ef][0]+value, ef_loc[ef][1]+value/2, ef_loc[ef][2]])
            if ef is 'FL' or ef is 'BL':
                move_ef(ef, [ef_loc[ef][0]-value, ef_loc[ef][1]-value/2, ef_loc[ef][2]])


def rotation_torso(direction, value):
    if direction is 'up':
        for ef in ef_loc.keys():
                if ef is 'FR' or ef is 'FL':
                    move_ef(ef, [ef_loc[ef][0], ef_loc[ef][1]+value, ef_loc[ef][2]-value*2])
                if ef is 'BR' or ef is 'BL':
                    move_ef(ef, [ef_loc[ef][0], ef_loc[ef][1]-value, ef_loc[ef][2]-value*2])

    if direction is 'down':
        for ef in ef_loc.keys():
                if ef is 'FR' or ef is 'FL':
                    move_ef(ef, [ef_loc[ef][0], ef_loc[ef][1]-value, ef_loc[ef][2]+value*2])
                if ef is 'BR' or ef is 'BL':
                    move_ef(ef, [ef_loc[ef][0], ef_loc[ef][1]+value, ef_loc[ef][2]+value*2])

    # if direction is 'fwd':
    #     for ef in ef_loc.keys():
    #         move_ef(ef, [ef_loc[ef][0], ef_loc[ef][1], ef_loc[ef][2] + value])
    #
    # if direction is 'bkwd':
    #     for ef in ef_loc.keys():
    #         move_ef(ef, [ef_loc[ef][0], ef_loc[ef][1], ef_loc[ef][2] - value])
    #
    if direction is 'right':
        for ef in ef_loc.keys():
            if ef is 'FR':
                move_ef(ef, [ef_loc[ef][0]+value, ef_loc[ef][1]+value/2, ef_loc[ef][2]-value])
            if ef is 'BR':
                move_ef(ef, [ef_loc[ef][0]-value, ef_loc[ef][1], ef_loc[ef][2]-value])
            if ef is 'FL':
                move_ef(ef, [ef_loc[ef][0]-value, ef_loc[ef][1], ef_loc[ef][2]+value])
            if ef is 'BL':
                move_ef(ef, [ef_loc[ef][0]+value, ef_loc[ef][1]+value/2, ef_loc[ef][2]+value])

    if direction is 'left':
        for ef in ef_loc.keys():
            if ef is 'FR':
                move_ef(ef, [ef_loc[ef][0]-value, ef_loc[ef][1]-value/2, ef_loc[ef][2]+value])
            if ef is 'BR':
                move_ef(ef, [ef_loc[ef][0]+value, ef_loc[ef][1], ef_loc[ef][2]+value])
            if ef is 'FL':
                move_ef(ef, [ef_loc[ef][0]+value, ef_loc[ef][1], ef_loc[ef][2]-value])
            if ef is 'BL':
                move_ef(ef, [ef_loc[ef][0]-value, ef_loc[ef][1]-value/2, ef_loc[ef][2]-value])




def generate_waypoints(center_x, center_y, step_height, step_width, waypoints_num):
    # waypoints_num = 10
    interp_x = [center_x - step_width / 2, center_x, center_x + step_width / 2]
    interp_y = [center_y + step_height, center_y, center_y + step_height]

    f = interpolate.interp1d(interp_x, interp_y, kind='quadratic')
    x_waypoints = np.linspace(center_x - step_width / 2, center_x + step_width / 2, waypoints_num)
    y_waypoints = f(x_waypoints)

    reverse_x_waypoints = np.linspace(x_waypoints[-1], x_waypoints[0], waypoints_num)
    reverse_y_waypoints = np.linspace(y_waypoints[-1], y_waypoints[0], waypoints_num)

    return x_waypoints, y_waypoints, reverse_x_waypoints, reverse_y_waypoints

def pre_walk_homeing():
    pass


def walk(direction):
    if direction == 'fwd':
        waypoints_num = 10
        z_offset = 1
        #-----------------------------
        # k_p = 0.01
        # acc_data = read_acc_data()
        # temp_center_x = k_p * -acc_data
        # print(acc_data)
        # print(temp_center_x)
        #-----------------------------
        x_waypoints, y_waypoints, reverse_x_waypoints, reverse_y_waypoints = generate_waypoints(center_x=0, center_y=0.14, step_height=0.05, step_width=0.1, waypoints_num=waypoints_num)
        for waypoints in range(waypoints_num):
            move_ef('FR', [-0.0439*z_offset, y_waypoints[waypoints], x_waypoints[waypoints]])
            move_ef('BL', [-0.0439*z_offset, y_waypoints[waypoints], x_waypoints[waypoints]])
            move_ef('FL', [-0.0439*z_offset, reverse_y_waypoints[waypoints], reverse_x_waypoints[waypoints]])
            move_ef('BR', [-0.0439*z_offset, reverse_y_waypoints[waypoints], reverse_x_waypoints[waypoints]])
        for waypoints in range(waypoints_num):
            move_ef('FL', [-0.0439*z_offset, y_waypoints[waypoints], x_waypoints[waypoints]])
            move_ef('BR', [-0.0439*z_offset, y_waypoints[waypoints], x_waypoints[waypoints]])
            move_ef('FR', [-0.0439*z_offset, reverse_y_waypoints[waypoints], reverse_x_waypoints[waypoints]])
            move_ef('BL', [-0.0439*z_offset, reverse_y_waypoints[waypoints], reverse_x_waypoints[waypoints]])

    elif direction == 'bkwd':
        waypoints_num = 10
        z_offset = 1
        #-----------------------------
        # k_p = 0.005
        # acc_data = read_acc_data()
        # temp_center_x = k_p * -acc_data
        # print(acc_data)
        # print(temp_center_x)
        #-----------------------------
        x_waypoints, y_waypoints, reverse_x_waypoints, reverse_y_waypoints = generate_waypoints(center_x=0, center_y=0.14, step_height=0.05, step_width=0.1, waypoints_num=waypoints_num)
        x_waypoints = np.flip(x_waypoints)
        y_waypoints = np.flip(y_waypoints)
        reverse_x_waypoints = np.flip(reverse_x_waypoints)
        reverse_y_waypoints = np.flip(reverse_y_waypoints)
        for waypoints in range(waypoints_num):
            move_ef('FL', [-0.0439*z_offset, y_waypoints[waypoints], x_waypoints[waypoints]])
            move_ef('BR', [-0.0439*z_offset, y_waypoints[waypoints], x_waypoints[waypoints]])
            move_ef('FR', [-0.0439*z_offset, reverse_y_waypoints[waypoints], reverse_x_waypoints[waypoints]])
            move_ef('BL', [-0.0439*z_offset, reverse_y_waypoints[waypoints], reverse_x_waypoints[waypoints]])
        for waypoints in range(waypoints_num):
            move_ef('FR', [-0.0439*z_offset, y_waypoints[waypoints], x_waypoints[waypoints]])
            move_ef('BL', [-0.0439*z_offset, y_waypoints[waypoints], x_waypoints[waypoints]])
            move_ef('FL', [-0.0439*z_offset, reverse_y_waypoints[waypoints], reverse_x_waypoints[waypoints]])
            move_ef('BR', [-0.0439*z_offset, reverse_y_waypoints[waypoints], reverse_x_waypoints[waypoints]])

    elif direction == 'right':
        waypoints_num = 10
        z_offset = 1
        x_waypoints, y_waypoints, reverse_x_waypoints, reverse_y_waypoints = generate_waypoints(center_x=0, center_y=0.14, step_height=0.05, step_width=0.1, waypoints_num=waypoints_num)
        for waypoints in range(waypoints_num):
            move_ef('FL', [x_waypoints[waypoints]-0.0439, y_waypoints[waypoints], 0])
            move_ef('BR', [-x_waypoints[waypoints]-0.0439, y_waypoints[waypoints], 0])
            move_ef('FR', [-reverse_x_waypoints[waypoints]-0.0439, reverse_y_waypoints[waypoints], 0])
            move_ef('BL', [reverse_x_waypoints[waypoints]-0.0439, reverse_y_waypoints[waypoints], 0])
        for waypoints in range(waypoints_num):
            move_ef('FR', [-x_waypoints[waypoints]-0.0439, y_waypoints[waypoints], 0])
            move_ef('BL', [x_waypoints[waypoints]-0.0439, y_waypoints[waypoints], 0])
            move_ef('FL', [reverse_x_waypoints[waypoints]-0.0439, reverse_y_waypoints[waypoints], 0])
            move_ef('BR', [-reverse_x_waypoints[waypoints]-0.0439, reverse_y_waypoints[waypoints], 0])

    elif direction == 'left':
        waypoints_num = 10
        z_offset = 1
        x_waypoints, y_waypoints, reverse_x_waypoints, reverse_y_waypoints = generate_waypoints(center_x=0, center_y=0.14, step_height=0.05, step_width=0.1, waypoints_num=waypoints_num)
        for waypoints in range(waypoints_num):
            move_ef('FL', [-x_waypoints[waypoints]-0.0439, y_waypoints[waypoints], 0])
            move_ef('BR', [x_waypoints[waypoints]-0.0439, y_waypoints[waypoints], 0])
            move_ef('FR', [reverse_x_waypoints[waypoints]-0.0439, reverse_y_waypoints[waypoints], 0])
            move_ef('BL', [-reverse_x_waypoints[waypoints]-0.0439, reverse_y_waypoints[waypoints], 0])
        for waypoints in range(waypoints_num):
            move_ef('FR', [x_waypoints[waypoints]-0.0439, y_waypoints[waypoints], 0])
            move_ef('BL', [-x_waypoints[waypoints]-0.0439, y_waypoints[waypoints], 0])
            move_ef('FL', [-reverse_x_waypoints[waypoints]-0.0439, reverse_y_waypoints[waypoints], 0])
            move_ef('BR', [reverse_x_waypoints[waypoints]-0.0439, reverse_y_waypoints[waypoints], 0])

    elif direction == 'rotate_right':
        waypoints_num = 10
        z_offset = 1
        x_waypoints, y_waypoints, reverse_x_waypoints, reverse_y_waypoints = generate_waypoints(center_x=0, center_y=0.14, step_height=0.05, step_width=0.1, waypoints_num=waypoints_num)
        for waypoints in range(waypoints_num):
            move_ef('FL', [x_waypoints[waypoints]-0.0439, y_waypoints[waypoints], 0])
            move_ef('BR', [x_waypoints[waypoints]-0.0439, y_waypoints[waypoints], 0])
            move_ef('FR', [-reverse_x_waypoints[waypoints]-0.0439, reverse_y_waypoints[waypoints], 0])
            move_ef('BL', [-reverse_x_waypoints[waypoints]-0.0439, reverse_y_waypoints[waypoints], 0])
        for waypoints in range(waypoints_num):
            move_ef('FR', [-x_waypoints[waypoints]-0.0439, y_waypoints[waypoints], 0])
            move_ef('BL', [-x_waypoints[waypoints]-0.0439, y_waypoints[waypoints], 0])
            move_ef('FL', [reverse_x_waypoints[waypoints]-0.0439, reverse_y_waypoints[waypoints], 0])
            move_ef('BR', [reverse_x_waypoints[waypoints]-0.0439, reverse_y_waypoints[waypoints], 0])

    elif direction == 'rotate_left':
        waypoints_num = 10
        z_offset = 1
        x_waypoints, y_waypoints, reverse_x_waypoints, reverse_y_waypoints = generate_waypoints(center_x=0, center_y=0.14, step_height=0.05, step_width=0.1, waypoints_num=waypoints_num)
        for waypoints in range(waypoints_num):
            move_ef('FL', [-x_waypoints[waypoints]-0.0439, y_waypoints[waypoints], 0])
            move_ef('BR', [-x_waypoints[waypoints]-0.0439, y_waypoints[waypoints], 0])
            move_ef('FR', [reverse_x_waypoints[waypoints]-0.0439, reverse_y_waypoints[waypoints], 0])
            move_ef('BL', [reverse_x_waypoints[waypoints]-0.0439, reverse_y_waypoints[waypoints], 0])
        for waypoints in range(waypoints_num):
            move_ef('FR', [x_waypoints[waypoints]-0.0439, y_waypoints[waypoints], 0])
            move_ef('BL', [x_waypoints[waypoints]-0.0439, y_waypoints[waypoints], 0])
            move_ef('FL', [-reverse_x_waypoints[waypoints]-0.0439, reverse_y_waypoints[waypoints], 0])
            move_ef('BR', [-reverse_x_waypoints[waypoints]-0.0439, reverse_y_waypoints[waypoints], 0])

# home_position()

for i in range(100000000000):


    # p.setJointMotorControl2(robot, 0, p.POSITION_CONTROL, -np.deg2rad(p.readUserDebugParameter(slide0)), force=10)
    # p.setJointMotorControl2(robot, 1, p.POSITION_CONTROL, np.deg2rad(p.readUserDebugParameter(slide1)), force=10)
    # p.setJointMotorControl2(robot, 2, p.POSITION_CONTROL, np.deg2rad(p.readUserDebugParameter(slide2)), force=10)
    # # #
    # p.setJointMotorControl2(robot, 4, p.POSITION_CONTROL, np.deg2rad(p.readUserDebugParameter(slide4)), force=10)
    # p.setJointMotorControl2(robot, 5, p.POSITION_CONTROL, np.deg2rad(p.readUserDebugParameter(slide5)), force=10)
    # p.setJointMotorControl2(robot, 6, p.POSITION_CONTROL, np.deg2rad(p.readUserDebugParameter(slide6)), force=10)
    # # #
    # p.setJointMotorControl2(robot, 8, p.POSITION_CONTROL, -np.deg2rad(p.readUserDebugParameter(slide8)), force=10)
    # p.setJointMotorControl2(robot, 9, p.POSITION_CONTROL, -np.deg2rad(p.readUserDebugParameter(slide9)), force=10)
    # p.setJointMotorControl2(robot, 10, p.POSITION_CONTROL, np.deg2rad(p.readUserDebugParameter(slide10)), force=10)
    # # #
    # p.setJointMotorControl2(robot, 12, p.POSITION_CONTROL, np.deg2rad(p.readUserDebugParameter(slide12)), force=10)
    # p.setJointMotorControl2(robot, 13, p.POSITION_CONTROL, -np.deg2rad(p.readUserDebugParameter(slide13)), force=10)
    # p.setJointMotorControl2(robot, 14, p.POSITION_CONTROL, np.deg2rad(p.readUserDebugParameter(slide14)), force=10)

    # p.addUserDebugText('target', [p.readUserDebugParameter(slide0),p.readUserDebugParameter(slide1), p.readUserDebugParameter(slide2)], lifeTime=1)
    p.stepSimulation()
    time.sleep(1./240.)
    key = p.getKeyboardEvents()
    # print(key)
    if 104 in key:
        home_position()
    elif 65297 in key:
        # translate_torso('up', 0.001)
        walk('fwd')
    elif 65298 in key:
        #translate_torso('down', 0.001)
        walk('bkwd')

    elif 65296 in key:
        walk('right')
    elif 65295 in key:
        walk('left')
    elif 65299 in key:
        walk('rotate_right')
    elif 65302 in key:
       walk('rotate_left')

    #

    # print('Y:', round(math.degrees(y),2))
    # print('Z:', round(math.degrees(z),2))
cubePos, cubeOrn = p.getBasePositionAndOrientation(robot)
print(cubePos,cubeOrn)
p.disconnect()
