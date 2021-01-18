#!/usr/bin/env python

import rospy

from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs import point_cloud2 as pc2c

import sys, select, os

if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

e = """
Communications Failed
"""

mensaje = pc2

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def pointCallback(points):
    global mensaje
    mensaje = points


if __name__ == "__main__":
    
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node("catch_point_cloud_node")
    pub_tg = rospy.Publisher("catch_point_cloud_tg",pc2,queue_size=10)
    pub_src = rospy.Publisher("catch_point_cloud_src",pc2,queue_size=10)
    sub = rospy.Subscriber("/camera/depth_registered/points", pc2, pointCallback)

    try:
        while(1):
            key = getKey()
            if key == 't':
                pub_tg.publish(mensaje)
            if key == 's':
                pub_src.publish(mensaje)
            else:
                if (key == '\x03'):
                    break
    except:
        print(e)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        
    



