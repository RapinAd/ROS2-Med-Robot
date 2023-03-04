import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import RPi.GPIO as GPIO
import time

class VelocitySubscriber(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("cmd_vel_subscriber")  # MODIFY NAME
        self.subscriber_ = self.create_subscription(
            Twist, "cmd_vel", self.cmd_to_pwm_callback, 10)
        self.get_logger().info("--Command Velocity Node has been started--")
        #PIN NUMBER
        self.right_motor_dir = 5
        self.right_motor_pwm = 6

        self.left_motor_dir = 16
        self.left_motor_pwm = 26

        #GPIO SETUP
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.right_motor_dir , GPIO.OUT)
        GPIO.setup(self.right_motor_pwm , GPIO.OUT)

        GPIO.setup(self.left_motor_dir , GPIO.OUT)
        GPIO.setup(self.left_motor_pwm , GPIO.OUT)

        self.pwm_r = GPIO.PWM(self.right_motor_pwm , 1000)
        self.pwm_l = GPIO.PWM(self.left_motor_pwm , 1000)

        self.pwm_r.start(0)
        self.pwm_l.start(0)

    def forward(self,vel_R,vel_L):
        print("forward")
        self.pwm_r.ChangeDutyCycle(0.5)
        self.pwm_l.ChangeDutyCycle(0.5)
        GPIO.output(self.right_motor_dir ,GPIO.LOW)
        GPIO.output(self.left_motor_dir,GPIO.LOW)

    def reverse(self,vel_R,vel_L):
        print("reverse")
        self.pwm_r.ChangeDutyCycle(0.5)
        self.pwm_l.ChangeDutyCycle(0.5)
        GPIO.output(self.right_motor_dir ,GPIO.HIGH)
        GPIO.output(self.left_motor_dir,GPIO.HIGH)

    def left(self,vel_R,vel_L):
        print("left")
        self.pwm_r.ChangeDutyCycle(0.5)
        self.pwm_l.ChangeDutyCycle(0.5)
        GPIO.output(self.right_motor_dir, GPIO.LOW)
        GPIO.output(self.left_motor_dir,GPIO.HIGH)

    def right(self,vel_R,vel_L):
        print("right")
        self.pwm_r.ChangeDutyCycle(0.5)
        self.pwm_l.ChangeDutyCycle(0.5)
        GPIO.output(self.right_motor_dir, GPIO.HIGH)
        GPIO.output(self.left_motor_dir,GPIO.LOW)

    def stop(self):
        print("stop")
        self.pwm_r.ChangeDutyCycle(0)
        self.pwm_l.ChangeDutyCycle(0)

    def cmd_to_pwm_callback(self, msg):
        right_wheel_vel = (msg.linear.x - msg.angular.z)/2
        left_wheel_vel = (msg.linear.x + msg.angular.z)/2

        print(right_wheel_vel, "/", left_wheel_vel)

        if right_wheel_vel > 0 and left_wheel_vel > 0:
            self.forward(right_wheel_vel,left_wheel_vel)
        elif right_wheel_vel < 0 and left_wheel_vel < 0:
            self.reverse(-right_wheel_vel,-left_wheel_vel)
        elif right_wheel_vel > 0 and left_wheel_vel < 0:
            self.left(right_wheel_vel,-left_wheel_vel)
        elif right_wheel_vel < 0 and left_wheel_vel > 0:
            self.right(-right_wheel_vel,left_wheel_vel)
        else:
            self.stop()
        
        


def main(args=None):
    rclpy.init(args=args)
    node = VelocitySubscriber()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
