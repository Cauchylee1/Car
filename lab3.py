#!/usr/bin/python3
# coding=utf8
# Date:2022/05/30import sys
import sys
import cv2
import time
import math
import rospy
import numpy as np
from threading import RLock, Timer, Thread

from std_srvs.srv import *
from std_msgs.msg import *
from sensor_msgs.msg import Image

from sensor.msg import Led
from chassis_control.msg import *
from visual_processing.msg import Result
from visual_processing.srv import SetParam
from intelligent_transport.srv import SetTarget
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

from armpi_pro import PID
from armpi_pro import Misc
from armpi_pro import bus_servo_control
from kinematics import ik_transform

lock = RLock(),
ik = ik_transform.ArmIK()

steps = ['forward', 'grasp', 'turnAround', 'forward', 'place','turnAround','forward', 'grasp', 'turnAround', 'forward', 'place','turnAround','forward', 'grasp', 'turnAround', 'forward', 'place']
cur_step = 0
arm_move = False
target_color = 'None'
__isRunning = False

# color for LED
range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
}

x_dis = 500  # 6th steering engine initial angle
y_dis = 0.15  # a magic number for ik solution
color_center_x = 0
color_center_y = 0
apriltag_center_x = 0
apriltag_center_y = 0
centreX = 320
centreY = 410
offset_y = 0
object_angle = 0
angle = 0
tag_id = 0
flag = False

# cube tracking
color_x_pid = PID.PID(P=0.06, I=0, D=0)
color_y_pid = PID.PID(P=0.00003, I=0, D=0)
apriltag_x_pid = PID.PID(P=0.06, I=0, D=0)  # pid初始化
apriltag_y_pid = PID.PID(P=0.00003, I=0, D=0)
result_sub = None
heartbeat_timer = None

num = 0


def run(msg):
    global lock
    global arm_move
    global steps, cur_step
    global color_center_x, color_center_y
    global apriltag_center_x, apriltag_center_y, object_angle, tag_id

    # rospy.loginfo("center_x = %f,center_y = %f",center_x,center_y)

    # rospy.loginfo("cur_step: %d",cur_step)
    with lock:
        if steps[cur_step] == 'grasp' and not arm_move:
            color_center_x = msg.center_x
            color_center_y = msg.center_y
        elif steps[cur_step] == 'place' and not arm_move:
            apriltag_center_x = msg.center_x
            apriltag_center_y = msg.center_y
            object_angle = msg.angle
            tag_id = msg.data


def move():
    global cur_step, steps
    global x_dis, y_dis
    global arm_move, flag
    global offset_y, object_angle, angle
    global __isRunning
    global target_color
    global num
    global color_center_x, color_center_y
    global apriltag_center_x, apriltag_center_y, tag_id

    # rospy.loginfo('target_color = %s',target_color)

    while __isRunning:
        # rospy.loginfo("cur_step: %d",cur_step)
        # forward to the target
        if steps[cur_step] == 'forward':
            # 85cm = 480 unit thus 5 unit for 1 cm
            # run 5s,thus velocity should be 85
            set_velocity.publish(85.0,90.0,0.0)
            rospy.sleep(10.45)
            # stop
            set_velocity.publish(0,0,0)
            # next step
            flag = True
            cur_step += 1
        elif steps[cur_step] == 'grasp':
            if flag:
                x_dis = 500
                y_dis = 0.15
                flag = False
                visual_running('color', target_color)

            diff_x = abs(color_center_x - centreX)
            diff_y = abs(color_center_y - centreY)

            if diff_x < 10:
                color_x_pid.SetPoint = color_center_x
            else:
                color_x_pid.SetPoint = centreX
            color_x_pid.update(color_center_x)
            dx = color_x_pid.output
            rospy.loginfo("dx: %f", dx)
            x_dis += int(dx)
            x_dis = 200 if x_dis < 200 else x_dis
            x_dis = 800 if x_dis > 800 else x_dis

            if diff_y < 10:
                color_y_pid.SetPoint = color_center_y
            else:
                color_y_pid.SetPoint = centreY
            color_y_pid.update(color_center_y)
            dy = color_y_pid.output
            rospy.loginfo("dy: %f", dy)
            y_dis += dy
            y_dis = 0.12 if y_dis < 0.12 else y_dis
            y_dis = 0.28 if y_dis > 0.28 else y_dis

            target = ik.setPitchRanges((0, round(y_dis, 4), 0.03), -180, -180, 0)

            if target:
                servo_data = target[1]
                rospy.loginfo("sevo_data[3]: %d,servo_data[4]: %d,servo_data[5]: %d,x_dis: %d ", servo_data['servo3'],
                              servo_data['servo4'], servo_data['servo5'], x_dis)
                bus_servo_control.set_servos(joints_pub, 20, ((3, servo_data['servo3']), (4, servo_data['servo4']),
                                                              (5, servo_data['servo5']), (6, x_dis)))
                rospy.sleep(0.02)
            if abs(dx) < 2 and abs(dy) < 0.003:
                num += 1
                if num == 10:
                    num = 0
                    # rospy.loginfo("ready to grasp")
                    offset_y = Misc.map(target[2], -180, -150, -0.03, 0.03)
                    arm_move = True  # ready to grasp
            else:
                num = 0

            if arm_move:
                buzzer_pub.publish(0.1)
                bus_servo_control.set_servos(joints_pub, 500, ((1, 20),))  # open claw
                rospy.sleep(0.5)
                print(offset_y)
                target = ik.setPitchRanges((0, round(y_dis, 4), -0.08), -180, -180, 0)  # arm down
                if target:
                    servo_data = target[1]
                    rospy.loginfo("sevo_data[3]: %d,servo_data[4]: %d,servo_d ata[5]: %d,x_dis: %d",
                                  servo_data['servo3'], servo_data['servo4'], servo_data['servo5'], x_dis)
                    bus_servo_control.set_servos(joints_pub, 1000,
                                                 ((3, servo_data['servo3']), (4, servo_data['servo4']),
                                                  (5, servo_data['servo5']), (6, x_dis)))
                rospy.sleep(1.5)
                bus_servo_control.set_servos(joints_pub, 500, ((1, 450),))  # close claw
                rospy.sleep(0.8)
                bus_servo_control.set_servos(joints_pub, 1500,
                                             ((1, 450), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))  # 机械臂抬起来
                rospy.sleep(1.5)
                arm_move = False
                # next step
                cur_step += 1
        elif steps[cur_step] == 'turnAround':
            # -0.55 * 2 = -0.1 = 90deg, thus -1.1 * 2 = -2.2 = 180 deg
            set_velocity.publish(0.0, 90.0, -0.55)
            rospy.sleep(4.0)
            # stop
            set_velocity.publish(0.0, 0.0, 0.0)
            rospy.sleep(1.0)
            # next step
            cur_step += 1
        elif steps[cur_step] == 'place':
            if flag:
                x_dis = 500
                y_dis = -0.15
                flag = False

            target = ik.setPitchRanges((0, y_dis, 0.0), -180, -180, 0)
            if target:
                servo_data = target[1]
                rospy.loginfo("sevo_data[3]: %d,servo_data[4]: %d,servo_data[5]: %d,x_dis: %d ", servo_data['servo3'],
                              servo_data['servo4'], servo_data['servo5'], x_dis)
                bus_servo_control.set_servos(joints_pub, 500, ((3, servo_data['servo3']), (4, servo_data['servo4']),
                                                               (5, servo_data['servo5']),(6, x_dis)))
                rospy.sleep(0.5)

            target = ik.setPitchRanges((0, y_dis, -0.08), -180, -180, 0)  # arm down
            if target:
                servo_data = target[1]
                bus_servo_control.set_servos(joints_pub, 1000, ((3, servo_data['servo3']), (4, servo_data['servo4']),
                                                                (5, servo_data['servo5'])))
            rospy.sleep(1.0)
            buzzer_pub.publish(0.1)
            bus_servo_control.set_servos(joints_pub, 500, ((1, 120),))  # open claw
            rospy.sleep(0.5)

            # 机械臂复位
            bus_servo_control.set_servos(joints_pub, 1500,
                                         ((1, 120), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))  # 机械臂抬起来
            rospy.sleep(1.5)

            # the end
            break
    reset()


def init():
    rospy.loginfo('Lab2 Init')
    initMove()
    reset()


def initMove(delay=True):
    with lock:
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 75), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))
    if delay:
        rospy.sleep(2)


def reset():
    global x_dis, y_dis
    global cur_step
    global target_color
    global arm_move, flag
    global color_center_x, color_center_y
    global apriltag_center_x, apriltag_center_y

    with lock:
        cur_step = 0
        target_color = 'None'
        arm_move = False
        flag = False
        color_x_pid.clear()
        color_y_pid.clear()
        apriltag_x_pid.clear()
        apriltag_y_pid.clear()
        off_rgb()
        x_dis = 500
        y_dis = 0.15
        color_center_x = 0
        color_center_y = 0
        apriltag_center_x = 0
        apriltag_center_y = 0
        set_velocity.publish(0, 90, 0)


def off_rgb():
    led = Led()
    led.index = 0
    led.rgb.r = 0
    led.rgb.g = 0
    led.rgb.b = 0
    rgb_pub.publish(led)
    led.index = 1
    rgb_pub.publish(led)


def set_rgb(color):
    global lock
    with lock:
        led = Led()
        led.index = 0
        led.rgb.r = range_rgb[color][2]
        led.rgb.g = range_rgb[color][1]
        led.rgb.b = range_rgb[color][0]
        rgb_pub.publish(led)
        rospy.sleep(0.05)
        led.index = 1
        rgb_pub.publish(led)
        rospy.sleep(0.05)


def set_running(msg):
    if msg.data:
        start_running()
    else:
        stop_running()

    return [True, 'set_running']


def set_target(msg):
    global lock
    global target_color

    rospy.loginfo('%s', msg)
    with lock:
        target_color = msg.data
        led = Led()
        led.index = 0
        led.rgb.r = range_rgb[target_color][2]
        led.rgb.g = range_rgb[target_color][1]
        led.rgb.b = range_rgb[target_color][0]
        rgb_pub.publish(led)
        led.index = 1
        rgb_pub.publish(led)
        rospy.sleep(0.1)
    return [True, 'set_target']


def stop_running():
    global lock
    global __isRunning

    rospy.loginfo('stop running Lab2')
    with lock:
        __isRunning = False
        reset()
        initMove(delay=False)
        # set_velocity.publish(0, 0, 0)
        rospy.ServiceProxy('/visual_processing/set_running', SetParam)()


def start_running():
    global lock
    global __isRunning

    rospy.loginfo('start running Lab2')
    with lock:
        # init()
        __isRunning = True
        rospy.sleep(0.1)
        th = Thread(target=move)
        th.setDaemon(True)
        th.start()


# enter service
def enter_func(msg):
    global lock
    global result_sub

    rospy.loginfo('enter object tracking')
    init()
    with lock:
        if result_sub is None:
            # wake up visual_processing_node
            rospy.ServiceProxy('/visual_processing/enter', Trigger)()
            # subscribe the result from the detection module of visual_processing_node
            result_sub = rospy.Subscriber('/visual_processing/result', Result, run)

    return [True, 'enter']


# exit service
def exit_func(msg):
    global lock
    global result_sub
    global __isRunning
    global heartbeat_timer

    rospy.loginfo('exit Lab2')
    with lock:
        __isRunning = False
        rospy.ServiceProxy('/visual_processing/exit', Trigger)()
        reset()
        try:
            if result_sub is not None:
                result_sub.unregister()
                result_sub = None
            if heartbeat_timer is not None:
                heartbeat_timer.cancel()
                heartbeat_timer = None
        except BaseException as e:
            rospy.loginfo('%s', e)

    return [True, 'exit']


# heartbeat connection service
def heartbeat_srv_cb(msg):
    global heartbeat_timer

    if isinstance(heartbeat_timer, Timer):
        heartbeat_timer.cancel()
    if msg.data:
        heartbeat_timer = Timer(5, rospy.ServiceProxy('/Lab2/exit', Trigger))
        heartbeat_timer.start()
    rsp = SetBoolResponse()
    rsp.success = msg.data

    return rsp


if __name__ == '__main__':
    # init node
    rospy.init_node('Lab2', log_level=rospy.DEBUG)
    # Steering Engine
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    # ServiceProxy
    visual_running = rospy.ServiceProxy('/visual_processing/set_running', SetParam)
    # Services
    enter_srv = rospy.Service('/Lab2/enter', Trigger, enter_func)
    running_srv = rospy.Service('/Lab2/set_running', SetBool, set_running)
    set_target_srv = rospy.Service('/Lab2/set_target', SetTarget, set_target)
    exit_srv = rospy.Service('/Lab2/exit', Trigger, exit_func)
    heartbeat_srv = rospy.Service('/Lab2/heartbeat', SetBool, heartbeat_srv_cb)
    # Publishers
    # Chassis Control
    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    set_translation = rospy.Publisher('/chassis_control/set_translation', SetTranslation, queue_size=1)
    # Buzzer Control
    buzzer_pub = rospy.Publisher('/sensor/buzzer', Float32, queue_size=1)
    # RGB LED Control
    rgb_pub = rospy.Publisher('/sensor/rgb_led', Led, queue_size=1)
    rospy.sleep(0.5)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down')
