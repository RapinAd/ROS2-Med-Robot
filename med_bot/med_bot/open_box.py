#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

import RPi.GPIO as GPIO
import time


class OpenBox(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("open_box")  # MODIFY NAME
        self.subscriber_ = self.create_subscription(
            Joy, "joy", self.open_box_callback, 10)
        self.get_logger().info("--Open Box Node has been started--")
        #PIN NUMBER
        self.servo1_pin = 12
        self.servo2_pin = 16
        self.servo3_pin = 20
        self.servo4_pin = 21

        #GPIO SETUP
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.servo1_pin, GPIO.OUT)
        GPIO.setup(self.servo2_pin, GPIO.OUT)
        GPIO.setup(self.servo3_pin, GPIO.OUT)
        GPIO.setup(self.servo4_pin, GPIO.OUT)

        self.servo1 = GPIO.PWM(self.servo1_pin , 50)
        self.servo2 = GPIO.PWM(self.servo2_pin , 50)
        self.servo3 = GPIO.PWM(self.servo3_pin , 50)
        self.servo4 = GPIO.PWM(self.servo4_pin , 50)

        self.servo1.start(12)
        self.servo2.start(12)
        self.servo3.start(12)
        self.servo4.start(12)

        #BOX_STATUS
        self.box1_status = 0
        self.box2_status = 0
        self.box3_status = 0
        self.box4_status = 0

    def open(self, servo):
        print("Box is opening")
        duty = 12
        servo.start(duty)
        while duty >= 8.65:
            servo.ChangeDutyCycle(duty)
            duty = duty - 0.1
            time.sleep(0.02)
            servo.ChangeDutyCycle(0)
            time.sleep(0.03)

    def close(self, servo):
        print("Box is closing")
        duty = 8.65
        servo.start(duty)
        while duty <= 12:
            servo.ChangeDutyCycle(duty)
            duty = duty + 0.12
            time.sleep(0.02)
            servo.ChangeDutyCycle(0)
            time.sleep(0.03)

    def open_box_callback(self, msg):
        button1_status = msg.buttons[0]
        button2_status = msg.buttons[1]
        button3_status = msg.buttons[2]
        button4_status = msg.buttons[3]

        print("Button 1 Status: ", button1_status, "    |    Box 1 Status: ", self.box1_status)
        print("Button 2 Status: ", button2_status, "    |    Box 1 Status: ", self.box2_status)
        print("Button 3 Status: ", button3_status, "    |    Box 1 Status: ", self.box3_status)
        print("Button 4 Status: ", button4_status, "    |    Box 1 Status: ", self.box4_status)

        if button1_status == 1 and self.box1_status == 0:
            self.open(self.servo1) #status 0 --> close, 1 --> open
            self.box1_status = 1
        elif button1_status == 1 and self.box1_status == 1:
            self.close(self.servo1)
            self.box1_status = 0
        elif button1_status == 0 and self.box1_status == 0:
            self.servo1.ChangeDutyCycle(0)
        else:
            self.servo1.ChangeDutyCycle(0)

        if button2_status == 1 and self.box2_status == 0:
            self.open(self.servo2)
            self.box2_status = 1
        elif button2_status == 1 and self.box2_status == 1:
            self.close(self.servo2)
            self.box2_status = 0
        elif button2_status == 0 and self.box2_status == 0:
            self.servo2.ChangeDutyCycle(0)
        else:
            self.servo2.ChangeDutyCycle(0)

        if button3_status == 1 and self.box3_status == 0:
            self.open(self.servo3)
            self.box3_status = 1
        elif button3_status == 1 and self.box3_status == 1:
            self.close(self.servo3)
            self.box3_status = 0
        elif button3_status == 0 and self.box3_status == 0:
            self.servo3.ChangeDutyCycle(0)
        else:
            self.servo3.ChangeDutyCycle(0)

        if button4_status == 1 and self.box4_status == 0:
            self.open(self.servo4)
            self.box4_status = 1
        elif button4_status == 1 and self.box4_status == 1:
            self.close(self.servo4)
            self.box4_status = 0
        elif button4_status == 0 and self.box4_status == 0:
            self.servo4.ChangeDutyCycle(0)
        else:
            self.servo4.ChangeDutyCycle(0)

def main(args=None):
    rclpy.init(args=args)
    node = OpenBox()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
