#!/usr/bin/env python

import rospy
import time
import orienbus

from geometry_msgs.msg import Twist

kp = 10
kd = 0.1
ki = 0.01

MAX_SPEED = 500

#port = '/dev/ttyUSB0' # modbus port

# Create orienbus object with port name
#orienbus = orienbus.OrienBus(port)

# slave addresses
address_rot_lb = 13
address_rot_rb = 12
address_rot_lf = 11
address_rot_rf = 10

p_time = 0
m_time = 0.1

class Motor():
    def __init__(self, address, name):
        #self.motor = orienbus.initialize(address)
        self.name = name
        self.curr_err = 0
        self.prev_err = 0
        self.accu_err = 0
        self.speed = 0
        self.target = 0

    def adjust_speed(self):
        self.curr_error = self.target - self.speed
        self.accu_err += self.curr_err
        p = proportional(self.target, self.speed)
        d = derivative(self.curr_err, self.prev_err)
        i = integral(self.accu_err)
        speed = p + i + d
        if abs(speed) < abs(MAX_SPEED):
            #self.motor.writeSpeed(min(speed, MAX_SPEED))
            print("{}: {}".format(self.name, min(speed, MAX_SPEED)))
        else:
            if speed < 0:
                #self.motor.writeSpeed(-MAX_SPEED)
                print("{}: {}".format(self.name, -MAX_SPEED))
            else:
                #self.motor.writeSpeed(MAX_SPEED)
                print("{}: {}".format(self.name, MAX_SPEED))
        self.prev_err = self.curr_err

    def set_speed(self, speed):
        self.speed = speed

    def set_target(self, target):
        self.target = target


def encoder_pos(data): # Twist message, angles in degrees
    global m_time
    lf.set_speed(data.linear.x)
    lb.set_speed(data.linear.y)
    rf.set_speed(data.linear.z)
    rb.set_speed(data.angular.x)
    m_time = rospy.get_time()

def desired_pos(data): # Reading from cmd_vel twist message
    lf.set_target(data.linear.x)
    lb.set_target(data.linear.y)
    rf.set_target(data.linear.z)
    rb.set_target(data.angular.x)

def proportional(desired, actual):
    return kp * (desired - actual)

def derivative(curr, prev):
    return kd * (curr - prev) / (m_time - p_time)

def integral(accu):
    return ki * accu

'''
def desired_state(cmd):
    for motor in motor_ls:
        if cmd == "0":
            motor.set_target(0)
        elif cmd == '1':
            motor.set_target(-90)
        elif cmd == '2':
            motor.set_target(90)
'''

def controller(motor_ls):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for i in motor_ls:
            i.adjust_speed()
            p_time = m_time
        rate.sleep()

def run_node():
    rospy.init_node('steering_control')
    enc_sub = rospy.Subscriber("encoder_positions", Twist, encoder_pos)
    targets = rospy.Subscriber("target_angles", Twist, desired_pos)
    controller(motor_ls)

    rospy.spin()

if __name__ == "__main__":
    try:
        lf = Motor(address_rot_lf, "lf")
        lb = Motor(address_rot_lb, "lb")
        rf = Motor(address_rot_rf, "rf")
        rb = Motor(address_rot_rb, "rb")
        motor_ls = [lf, lb, rf, rb]
        run_node()

    except rospy.ROSInterruptException:
        pass
