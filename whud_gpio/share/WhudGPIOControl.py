#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import Jetson.GPIO as GPIO
import time
from std_msgs.msg import String
import threading

class GPIOControl:
    def __init__(self):
        #Subscriber Define
        self.gpio_ctrl_sub_ = rospy.Subscriber("/gpio_on_off", String, callback=self.GPIOCtrlCb)
        self.led_ctrl_sub_ = rospy.Subscriber("/led_on_off", String, callback=self.LedCtrlCb)
        self.buzzer_ctrl_sub_ = rospy.Subscriber("/buzzer_on_off", String, callback=self.BuzzerCtrlCb)

        #Publisher Define
        self.button_state_pub_ = rospy.Publisher("/botton_state", String, queue_size=1)

        #Params
        self.led_ctrl_pin_ = 22
        self.buzzer_ctrl_pin_ = 40
        self.botton_state_pin_ = 26
        self.botton_mode_pin_ = 24

        self.led_on_off_flag_ = False
        self.buzzer_on_off_flag_ = False
        self.gpio_on_off_flag_ = True

        self.pwm_freq_ = 500
        self.sleep_time_ = 0.5/float(self.pwm_freq_)

        #Initialize
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup([self.led_ctrl_pin_],GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup([self.buzzer_ctrl_pin_],GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup([self.botton_state_pin_, self.botton_mode_pin_],GPIO.IN)

        self.thread_pwm_ = threading.Thread(target=self.PWMOutput)
        self.thread_pwm_.start()

        GPIO.add_event_detect(self.botton_state_pin_, GPIO.RISING, callback=self.BottonStateCb)
        GPIO.add_event_detect(self.botton_mode_pin_, GPIO.RISING, callback=self.BottonModeCb)

    def GPIOSpin(self):
        while(self.gpio_on_off_flag_):
            if(self.led_on_off_flag_):
                GPIO.output([self.led_ctrl_pin_],GPIO.HIGH)
            else:
                GPIO.output([self.led_ctrl_pin_],GPIO.LOW)


    def LedCtrlCb(self,led_ctrl_msg):
        if(led_ctrl_msg.data == "on"):
            self.led_on_off_flag_ = True
        elif(led_ctrl_msg.data == "off"):
            self.led_on_off_flag_ = False
        else:
            print("Wrong Param in led_on_off")

    def BuzzerCtrlCb(self,buzzer_ctrl_msg):
        if(buzzer_ctrl_msg.data == "on"):
            self.buzzer_on_off_flag_ = True
        elif(buzzer_ctrl_msg.data == "off"):
            self.buzzer_on_off_flag_ = False
        else:
            print("Wrong Param in buzzer_on_off")     
    
    def GPIOCtrlCb(self, gpio_ctrl_msg):
        if(gpio_ctrl_msg.data == "on"):
            self.gpio_on_off_flag_ = True
        elif(gpio_ctrl_msg.data == "off"):
            self.gpio_on_off_flag_ = False
        else:
            print("Wrong Param in buzzer_on_off")          

    def BottonStateCb(self,data):
        count = 0
        print("Botton State Changed")
        while(count<5):
            count += 1
            self.button_state_pub_.publish("Start")

    def BottonModeCb(self,data):
        print("Botton Mode Changed")
        index = 0
        with open("/home/nvidia/Code/simple_ws/src/whud_state_machine/config/task_stage.txt","r+") as f:
            index=int(f.readline())
            f.close()
        with open("/home/nvidia/Code/simple_ws/src/whud_state_machine/config/task_stage.txt","w+") as f:
            index = index%4+1
            f.write(str(index))
            f.close()

        while(index > 0):
            index -= 1
            self.led_on_off_flag_ = True
            GPIO.output([self.led_ctrl_pin_],GPIO.HIGH)
            time.sleep(0.25)
            self.led_on_off_flag_ = False
            GPIO.output([self.led_ctrl_pin_],GPIO.LOW)
            time.sleep(0.25)
        GPIO.output(self.led_ctrl_pin_,GPIO.LOW)


    def PWMOutput(self):
        while(self.gpio_on_off_flag_):
            if(self.buzzer_on_off_flag_):
                GPIO.output(self.buzzer_ctrl_pin_, GPIO.HIGH)
                time.sleep(self.sleep_time_)
                GPIO.output(self.buzzer_ctrl_pin_, GPIO.LOW)
                time.sleep(self.sleep_time_)

    def GPIOCleanUp(self):
        GPIO.cleanup([self.led_ctrl_pin_, self.buzzer_ctrl_pin_, self.botton_state_pin_])
        

rospy.init_node("gpio_node",anonymous=True)
GPIONode = GPIOControl()
GPIONode.GPIOSpin()
GPIONode.GPIOCleanUp()

