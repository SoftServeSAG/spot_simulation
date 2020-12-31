#!/usr/bin/env python

from __future__ import print_function

import threading

# import roslib; roslib.load_manifest('teleop_rs_keyboard')
import rospy

from rs_msgs.msg import GaitInput

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(1,0,0,1),
        'l':(1,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        # spot_name = str(input("Tell me spot name: "))
        spot_name = "spot1"
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('/'+ spot_name + '/inverse_gait_input', GaitInput, queue_size=1)
        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.1
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.StepLength = 0.11
        self.LateralFraction = 0.0
        self.YawRate = 0.0
        self.StepVelocity = 1.1
        self.ClearanceHeight = 0.1
        self.PenetrationDepth = 0.003
        self.SwingPeriod = 0.25
        self.YawControl = 0.0
        self.speed = 0.0
        self.turn = 0.0
        
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()
        
    def update(self, x, y, z, roll, pitch, yaw, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, PenetrationDepth, SwingPeriod, YawControl, speed, turn):
        self.x = self.x
        self.y = self.y
        self.z = self.z
        self.roll = self.roll
        self.pitch = self.pitch
        self.yaw = self.yaw
        self.StepLength = StepLength * speed
        self.LateralFraction = self.LateralFraction
        self.YawRate = YawRate * turn
        self.StepVelocity = self.StepVelocity
        self.ClearanceHeight = self.ClearanceHeight
        self.PenetrationDepth = self.PenetrationDepth
        self.SwingPeriod = self.SwingPeriod
        self.YawControl = self.YawControl
        
    def publish_gait_msg(self):
        self.condition.acquire()
        # Wait for a new message or timeout.
        # self.condition.wait(self.timeout)
        gait_msg = GaitInput()
        gait_msg.x = float(self.x)
        gait_msg.y = float(self.y)
        gait_msg.z = float(self.z)
        gait_msg.roll = float(self.roll)
        gait_msg.pitch = float(self.pitch)
        gait_msg.yaw = float(self.yaw)
        gait_msg.StepLength = float( self.StepLength)
        gait_msg.LateralFraction = float(self.LateralFraction)
        gait_msg.YawRate = float(self.YawRate)
        gait_msg.StepVelocity = float(self.StepVelocity)
        gait_msg.ClearanceHeight = float(self.ClearanceHeight)
        gait_msg.PenetrationDepth = float(self.PenetrationDepth)
        gait_msg.SwingPeriod = float(self.SwingPeriod)
        gait_msg.YawControl = float(self.YawControl)
        
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()
        # Publish.
        self.publisher.publish(gait_msg)
        

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def stop(self):
        self.done = True
        self.update(0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.publish_gait_msg()
        self.join()

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.11)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)
    x = 0.0
    y = 0.0
    z = 0.0
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    StepLength = 0.0
    LateralFraction = 0.0
    YawRate = 0.0
    StepVelocity = 0.0
    ClearanceHeight = 0.0
    PenetrationDepth = 0.0
    SwingPeriod = 0.0
    YawControl = 0.0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, roll, pitch, yaw, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, PenetrationDepth, SwingPeriod, YawControl, speed, turn)

        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                StepLength = moveBindings[key][0]
                YawRate = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                # if key == '' and StepLength == 0 and YawRate == 0:
                #     continue
                x = 0.0
                y = 0.0
                z = 0.0
                roll = 0.0
                pitch = 0.0
                yaw = 0.0
                StepLength = 0.0
                LateralFraction = 0.0
                YawRate = 0.0
                StepVelocity = 0.0
                ClearanceHeight = 0.0
                PenetrationDepth = 0.0
                SwingPeriod = 0.0
                if (key == '\x03'):
                    StepLength = 0.0
                    break
            pub_thread.update(x, y, z, roll, pitch, yaw, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, PenetrationDepth, SwingPeriod, YawControl, speed, turn)
            pub_thread.publish_gait_msg()


    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
