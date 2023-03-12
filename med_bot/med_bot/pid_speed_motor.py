#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import numpy as np
import time

from gpiozero import PhaseEnableMotor, RotaryEncoder

class PIDController(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("PID_controller") # MODIFY NAME
        self.subscriber_ = self.create_subscription(
            Twist, "cmd_vel", self.PID_controller_callback, 10)
        self.get_logger().info("--PID CONTROLLER Node has been started--")
        
    
        # Define Pin
        self.left_motor_dir = 26
        self.left_motor_pwm = 13

        self.right_motor_dir = 6
        self.right_motor_pwm = 5

        self.left_encoder_a = 27
        self.left_encoder_b = 22

        self.right_encoder_a = 10
        self.right_encoder_b = 9

        # Pin Enable


        self.med_bot_L = PhaseEnableMotor(self.left_motor_dir, self.left_motor_pwm)
        self.med_bot_R = PhaseEnableMotor(self.right_motor_dir, self.right_motor_pwm)
        self.encoderL = RotaryEncoder(self.left_encoder_a, self.left_encoder_b, max_steps=0)
        self.encoderR = RotaryEncoder(self.right_encoder_a, self.right_encoder_b, max_steps=0)
        
        # Assigning parameter values
        self.ppr = 400  # Pulses Per Revolution of the encoder
        self.tstop = 20  # Loop execution duration (s)
        self.tsample = 0.02  # Sampling period for code execution (s)
        self.kp = 0.278 #0.278
        self.ki = 2 #2
        self.kd = 0


        # Global Variable
        self.tprev_L = 0
        self.tcurr_L = 0
        self.tdiff_L = 0
        self.step_curr_L = 0
        self.step_prev_L = 0
        self.step_diff_L = 0
        self.rpm_L = 0.0
        self.rpm_L_fil = 0.0
        self.rpm_L_prev = 0.0
        self.eintegral_L = 0.0
        self.dedt_L = 0.0 
        self.eL_prev = 0.0
        

        self.tprev_R = 0
        self.tcurr_R = 0
        self.tdiff_R = 0
        self.step_curr_R = 0
        self.step_prev_R = 0
        self.step_diff_R = 0
        self.rpm_R = 0.0
        self.rpm_R_fil = 0.0
        self.rpm_R_prev = 0.0
        self.eintegral_R = 0.0
        self.dedt_R = 0.0
        self.eR_prev = 0.0

        self.tstart = time.perf_counter()

    def pwm_drive(self, target_L, target_R):
        
        time.sleep(self.tsample)

        self.step_curr_L = self.encoderL.steps
        self.tcurr_L = time.perf_counter() - self.tstart
        self.tdiff_L = self.tcurr_L - self.tprev_L
        self.step_diff_L = self.step_curr_L - self.step_prev_L
        self.rpm_L = (-self.step_diff_L / self.tdiff_L) * 60 / self.ppr
        self.step_prev_L = self.step_curr_L
        self.tprev_L = self.tcurr_L

        self.step_curr_R = self.encoderR.steps
        self.tcurr_R = time.perf_counter() - self.tstart
        self.tdiff_R = self.tcurr_R - self.tprev_R
        self.step_diff_R = self.step_curr_R-self.step_prev_R
        self.rpm_R = (self.step_diff_R / self.tdiff_R) * 60 / self.ppr
        self.step_prev_R = self.step_curr_R
        self.tprev_R = self.tcurr_R

        # low-pass filter 25 Hz
        self.rpm_L_fil = 0.854*self.rpm_L_fil + 0.0728*self.rpm_L + 0.0728*self.rpm_L_prev
        self.rpm_L_prev = self.rpm_L

        self.rpm_R_fil = 0.854*self.rpm_R_fil + 0.0728*self.rpm_R + 0.0728*self.rpm_R_prev
        self.rpm_R_prev = self.rpm_R

        # signal u
        eL = (target_L - self.rpm_L_fil)
        eR = (target_R - self.rpm_R_fil)

        self.dedt_L = (eL-self.eL_prev)/self.tdiff_L
        self.dedt_R = (eR-self.eR_prev)/self.tdiff_R

        self.eintegral_L = self.eintegral_L + eL*self.tdiff_L
        self.eintegral_R = self.eintegral_R + eR*self.tdiff_R

        uL = (self.kp*eL + self.ki*self.eintegral_L + self.kd*self.dedt_L)/255
        uR = (self.kp*eR + self.ki*self.eintegral_R + self.kd*self.dedt_R)/255

        #print(self.rpm_L_fil, " ", self.tcurr_L," ", self.rpm_R_fil," ",self.tcurr_R)
            
        self.eL_prev = eL
        self.eR_prev = eR

        if uL > 0:
            if uL > 1:
                uL = 1
            self.med_bot_L.forward(uL)
        else:
            uL = np.abs(uL)
            if uL > 1:
                uL = 1
            self.med_bot_L.backward(uL)

        if uR > 0:
            if uR > 1:
                uR = 1
            self.med_bot_R.forward(uR)
        else:
            uR = np.abs(uR)
            if uR > 1:
                uR = 1
            self.med_bot_R.backward(uR)

    def PID_controller_callback(self, msg):
        right_wheel_vel = (msg.linear.x - msg.angular.z)/2
        left_wheel_vel = (msg.linear.x + msg.angular.z)/2



        if np.abs(right_wheel_vel) == np.abs(left_wheel_vel):
            if right_wheel_vel > 0 and left_wheel_vel > 0:
                # print ("--Go FORWARD--")
                target_L = 83
                target_R = 83
            elif right_wheel_vel < 0 and left_wheel_vel < 0:
                # print ("--Go BACKWARD--")
                target_L = -83
                target_R = -83
            elif right_wheel_vel > 0 and left_wheel_vel < 0:
                # print ("--TURN RIGHT--")
                target_L = 40
                target_R = -40
            elif right_wheel_vel < 0 and left_wheel_vel > 0:
                # print ("--TURN LEFT--")
                target_L = -40
                target_R = 40
            else:
                # print("--STOP--")
                target_L = 0
                target_R = 0
        else:
            if right_wheel_vel > 0 and left_wheel_vel > 0:
                if right_wheel_vel > left_wheel_vel:
                    # print ("--TURN RIGHT A BIT FORWARD--")
                    target_L = 83
                    target_R = 40
                else :
                    # print ("--TURN LEFT A BIT FORWARD--")
                    target_L = 40
                    target_R = 83
            elif right_wheel_vel < 0 and left_wheel_vel < 0:
                if right_wheel_vel > left_wheel_vel:
                    # print ("--TURN RIGHT A BIT BACKWARD--")
                    target_L = -83
                    target_R = -40
                else :
                    # print ("--TURN LEFT A BIT BACKWARD--")
                    target_L = -40                      
                    target_R = -83
        
        self.pwm_drive(target_L,target_R)


def main(args=None):
        rclpy.init(args=args)
        node = PIDController() # MODIFY NAME
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == "__main__":
    main()