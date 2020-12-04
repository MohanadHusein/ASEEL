import redboard
import math
import numpy as np
import time
import getch
from scipy import interpolate
from adxl345.i2c import ADXL345

acc = ADXL345(alternate=True)
acc.set_data_rate(800)
acc.set_range(16, True)
acc.power_on()

def read_acc_data(samples = 50):
        gx_buffer = []
        gy_buffer = []
        gz_buffer = []
        for x in range(50):
                acc_data = acc.read_data()
                gx_buffer.append(acc_data[0])
                gy_buffer.append(acc_data[1])
                gz_buffer.append(acc_data[2])
                time.sleep(0.0001)
        print(round(np.rad2deg(np.median(gx_buffer))+45,2), round(np.rad2deg(np.median(gy_buffer)),2), np.round(np.rad2deg(np.median(gz_buffer)),2))
        return [round(np.rad2deg(np.median(gx_buffer))+45,2), round(np.rad2deg(np.median(gy_buffer)),2), np.round(np.rad2deg(np.median(gz_buffer)),2)]


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
    #print('-----------------')
    #print(a1, a2, a3)

    if a1 > 90 or a1 < -90 or a2 > 120 or a2 < -120 or a3 > 150 or a3 < 30:
        return np.nan, np.nan, np.nan
    else:
        return a1, a2, a3


ef_loc = {'FR':[None, None, None], 'FL':[None, None, None], 'BL':[None, None, None], 'BR': [None, None, None]}

def home_position():
    move_ef('FR', [-0.0439*2, 0.2, 0])
    move_ef('FL', [-0.0439*2, 0.2, 0])
    move_ef('BL', [-0.0439*2, 0.2, 0])
    move_ef('BR', [-0.0439*2, 0.2, 0])

    #move_ef('FR', [-0.022, 0.3, -0.05])
    #move_ef('FL', [-0.022, 0.3, -0.05])
    #move_ef('BL', [-0.022, 0.3, -0.05])
    #move_ef('BR', [-0.022, 0.3, -0.05])


    #move_ef('FR', [0, 0.3, -0.05])
    #move_ef('FL', [0, 0.3, -0.05])
    #move_ef('BL', [0, 0.3, -0.05])
    #move_ef('BR', [0, 0.3, -0.05])


def move_ef(leg, new_loc):
    delay = 0.001
    a, b, c = calculate_x_y_z(new_loc[0], new_loc[1], new_loc[2])
    if not(np.isnan(a) or np.isnan(b) or np.isnan(c)):
        ef_dict = {'FR': ['FR0', 'FR1', 'FR2'],
                   'FL': ['FL0', 'FL1', 'FL2'],
                   'BL': ['BL0', 'BL1', 'BL2'],
                   'BR': ['BR0', 'BR1', 'BR2']}

        goto_angle(ef_dict[leg][0], a, delay)
        goto_angle(ef_dict[leg][1], b, delay)
        goto_angle(ef_dict[leg][2], c, delay)
        ef_loc[leg] = new_loc


def translate_torso(direction, value):
    if direction is 'up':
        for ef in ef_loc.keys():
            move_ef(ef, [ef_loc[ef][0], ef_loc[ef][1]+value, ef_loc[ef][2]])

    if direction is 'down':
        for ef in ef_loc.keys():
            move_ef(ef, [ef_loc[ef][0], ef_loc[ef][1]-value, ef_loc[ef][2]])

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


def goto_angle(servo, angle, speed=1):
    if servo == 'FR0':
        pwm = round(1410+(angle*7.4))
        redboard.servo20_P(pwm)
    elif servo == 'FR1':
        pwm = round(1440+(angle*7.4))
        redboard.servo21_P(pwm)
    elif servo == 'FR2':
        if angle >= 30 and angle <= 150:
            pwm = round(1470+(90*7.4)-(angle*7.4))
            redboard.servo22_P(pwm)
        else:
            print('ANGLE OUT OF RANGE!!')

    elif servo == 'BR0':
        pwm = round(1450-(angle*7.4))
        redboard.servo9_P(pwm)
    elif servo == 'BR1':
        pwm = round(1490+(angle*7.4))
        redboard.servo8_P(pwm)
    elif servo == 'BR2':
        if angle >= 30 and angle <= 150:
            pwm = round(1540 + (90 * 7.4) - (angle * 7.4))
            redboard.servo7_P(pwm)
        else:
            print('ANGLE OUT OF RANGE!!')

    elif servo == 'FL0':
        pwm = round(1360-(angle*7.4))
        redboard.servo6_P(pwm)
    elif servo == 'FL1':
        pwm = round(1450-(angle*7.4))
        redboard.servo13_P(pwm)
    elif servo == 'FL2':
        if angle >= 30 and angle <= 150:
            pwm = round(1570-(90*7.4)+(angle*7.4))
            redboard.servo27_P(pwm)
        else:
            print('ANGLE OUT OF RANGE!!')

    elif servo == 'BL0':
        pwm = round(1440+(angle*7.4))
        redboard.servo5_P(pwm)
    elif servo == 'BL1':
        pwm = round(1380-(angle*7.4))
        redboard.servo11_P(pwm)
    elif servo == 'BL2':
        if angle >= 30 and angle <= 150:
            pwm = round(1550-(90*7.4)+(angle*7.4))
            redboard.servo10_P(pwm)
        else:
            print('ANGLE OUT OF RANGE!!')


    # elif servo == 'test__':
    #     pwm = round(1520-(90*7.4)+(angle*7.4))
    #     redboard.servo10_P(pwm)

    time.sleep(speed)


def generate_waypoints(center_x, center_y, step_height, step_width, waypoints_num):
    #waypoints_num = 10
    interp_x = [center_x - step_width / 2, center_x, center_x + step_width / 2]
    interp_y = [center_y + step_height, center_y, center_y + step_height]

    f = interpolate.interp1d(interp_x, interp_y, kind='quadratic')
    x_waypoints = np.linspace(center_x - step_width / 2, center_x + step_width / 2, waypoints_num)
    y_waypoints = f(x_waypoints)

    reverse_x_waypoints = np.linspace(x_waypoints[-1], x_waypoints[0], waypoints_num)
    reverse_y_waypoints = np.linspace(y_waypoints[-1], y_waypoints[0], waypoints_num)

    return x_waypoints, y_waypoints, reverse_x_waypoints, reverse_y_waypoints

def walk(direction):
    if direction == 'fwd':
        z_offset = 1
        waypoints_num = 10
        x_waypoints, y_waypoints, reverse_x_waypoints, reverse_y_waypoints = generate_waypoints(center_x=0, center_y=0.14, step_height=0.05, step_width=0.1, waypoints_num=waypoints_num)
        for waypoints in range(waypoints_num):
            move_ef('FR', [-0.0439*z_offset, reverse_y_waypoints[waypoints], reverse_x_waypoints[waypoints]])
            move_ef('BL', [-0.0439*z_offset, reverse_y_waypoints[waypoints], reverse_x_waypoints[waypoints]])
            move_ef('FL', [-0.0439*z_offset, y_waypoints[waypoints], x_waypoints[waypoints]])
            move_ef('BR', [-0.0439*z_offset, y_waypoints[waypoints], x_waypoints[waypoints]])
        for waypoints in range(waypoints_num):
            move_ef('FL', [-0.0439*z_offset, reverse_y_waypoints[waypoints], reverse_x_waypoints[waypoints]])
            move_ef('BR', [-0.0439*z_offset, reverse_y_waypoints[waypoints], reverse_x_waypoints[waypoints]])
            move_ef('FR', [-0.0439*z_offset, y_waypoints[waypoints], x_waypoints[waypoints]])
            move_ef('BL', [-0.0439*z_offset, y_waypoints[waypoints], x_waypoints[waypoints]])


    elif direction == 'bkwd':
        waypoints_num = 10
        x_waypoints, y_waypoints, reverse_x_waypoints, reverse_y_waypoints = generate_waypoints(center_x=0, center_y=0.14, step_height=0.05, step_width=0.1, waypoints_num=waypoints_num)
        z_offset = 1
        x_waypoints = np.flip(x_waypoints)
        y_waypoints = np.flip(y_waypoints)
        reverse_x_waypoints = np.flip(reverse_x_waypoints)
        reverse_y_waypoints = np.flip(reverse_y_waypoints)
        for waypoints in range(waypoints_num):
            move_ef('FL', [-0.0439*z_offset, reverse_y_waypoints[waypoints], reverse_x_waypoints[waypoints]])
            move_ef('BR', [-0.0439*z_offset, reverse_y_waypoints[waypoints], reverse_x_waypoints[waypoints]])
            move_ef('FR', [-0.0439*z_offset, y_waypoints[waypoints], x_waypoints[waypoints]])
            move_ef('BL', [-0.0439*z_offset, y_waypoints[waypoints], x_waypoints[waypoints]])
        for waypoints in range(waypoints_num):
            move_ef('FR', [-0.0439*z_offset, reverse_y_waypoints[waypoints], reverse_x_waypoints[waypoints]])
            move_ef('BL', [-0.0439*z_offset, reverse_y_waypoints[waypoints], reverse_x_waypoints[waypoints]])
            move_ef('FL', [-0.0439*z_offset, y_waypoints[waypoints], x_waypoints[waypoints]])
            move_ef('BR', [-0.0439*z_offset, y_waypoints[waypoints], x_waypoints[waypoints]])

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


while(1):
    key = getch.getch()
    print(key)
    if key == 'h':
        home_position()
    elif key == 'w':
        #translate_torso('up', 0.01)
         walk('fwd')
    elif key == 's':
        #translate_torso('down', 0.01)
        walk('bkwd')
    elif key == 'd':
        walk('right')
    elif key == 'a':
        walk('left')
    elif key == 'q':
        #translate_torso('fwd', 0.01)
        walk('rotate_left')
    elif key == 'e':
        walk('rotate_right')
        # translate_torso('bkwd', 0.01)