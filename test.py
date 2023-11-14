#!/usr/bin/python3
# coding=utf8
import sys
import rospy
from chassis_control.msg import *
if sys.version_info.major == 2:
    print('run')
    sys.exit(0)

print('Test')

start = True
offset_ticks = 0
cur_i = 0


# before shut down
def stop():
    # similar to external in C
    global start

    start = False
    print('STop')
    set_velocity.publish(0, 0, 0)  # stop


# def activity(linV, directA, yawR):
#     # publish a chassis control msg, with linear velocity 60，direction angle 90，yaw rate 0(<0，clockwise)
#     set_velocity.publish(linV, directA, yawR)


if __name__ == '__main__':
    # 进入
    rospy.init_node('test', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    rospy.ServiceProxy('/object_tracking/set_running',SetBool)(True)#进入追踪
    rospy.ServiceProxy('/object_tracking/set_target',SetTarget)('blue')#设置追踪颜色
    rospy.sleep(16.0)
    rospy.ServiceProxy('/object_tracking/set_running', SetBool)(False)  # 结束追踪
    set_velocity.publish(0, 0, 0)  # 停下
    rospy.ServiceProxy('/visual_processing/enter', Trigger)()#进入智能抓取
    rospy.ServiceProxy('/visual_processing/set_running', Trigger)(True)  # 启动智能抓取
    set_velocity.publish(0.0, 90.0, -0.55)#旋转
    rospy.sleep(2.6)
    set_velocity.publish(0, 0, 0)  # 停下
    rospy.ServiceProxy('/object_tracking/set_running', SetBool)(True)  # 进入追踪
    rospy.ServiceProxy('/object_tracking/set_target', SetTarget)('blue')  # 设置追踪颜色
    rospy.sleep(16.0)
    rospy.ServiceProxy('/object_tracking/set_running', SetBool)(False)  # 结束追踪
    set_velocity.publish(0, 0, 0)  # 停下
    rospy.ServiceProxy('/visual_processing/set_running', Trigger)(False)  # 结束智能抓取
    rospy.ServiceProxy('/intelligent_grasp/exit',Trigger)()#退出智能抓取
    rospy.ServiceProxy('/object_tracking/exit',Trigger)() #退出追踪
    rospy.ServiceProxy('/visual_processing/exit', Trigger)()  # 退出
    print('Shut down')
