import sys
import cv2
import time
import math
import rospy
import numpy as np
from armpi_pro import Misc
from armpi_pro import apriltag
from threading import RLock, Timer
from std_srvs.srv import *
from std_msgs.msg import *
from sensor_msgs.msg import Image
from visual_processing.msg import Result
from visual_processing.srv import SetParam

lock = RLock()


result_sub = None

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

print('Bow Shaped Move Lab')

start = True
offset_ticks = 0
cur_i = 0


# before shut down
def stop():
    # similar to external in C
    global start

    start = False
    print('Shutting down...')
    set_velocity.publish(0, 0, 0)  # stop



result_sub = None
heartbeat_timer = None


if __name__ == '__main__':
    # 初始化节点
    rospy.init_node('test', log_level=rospy.DEBUG)

    rospy.on_shutdown(stop)  # stop callback function
    # Mc Wheel chassis control
    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)

    # the corresponding offset ticks for the chassis control
    # forward->turn right->forward->turn right->forward->turn left->forward->turn left->forward->turn right->forward
    ticks = [8.0, 1.3, 8.0]
    args = [
        [107.0, 90.0, 0.0],
        [0.0, 90.0, -1.375],
        [107.0, 90.0, 0.0]
    ]

    # without interrupt from the keyboard,do the routes
    route_steps_len = len(ticks)


    #第一步
    # get tick
    fin_tick = ticks[0]
    # get args
    arg = args[0]
    # act
    set_velocity.publish(arg[0], arg[1], arg[2])
    # actual tick inc
    rospy.sleep(1)



    #第二步
    rospy.ServiceProxy('/intelligent_grasp/enter',Trigger)()
    rospy.ServiceProxy('/intelligent_grasp/set_running', SetBool)(True)

    # actual tick inc
    rospy.sleep(1000)
    # get tick
    fin_tick = ticks[1]
    # get args
    arg = args[1]
    # act
    set_velocity.publish(arg[0], arg[1], arg[2])
    # actual tick inc
    rospy.sleep(1)


    #第三步
    # get tick
    fin_tick = ticks[0]
    # get args
    arg = args[0]
    # act
    set_velocity.publish(arg[0], arg[1], arg[2])
    # actual tick inc
    rospy.sleep(1)
    rospy.ServiceProxy('/intelligent_grasp/put',Trigger)()

    set_velocity.publish(0, 0, 0)  # stop
    print('Shut down')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    finally:
        cv2.destroyAllWindows()