from subprocess import run
import time
import RPi.GPIO as GPIO
import pigpio

from rpi_hardware_pwm import HardwarePWM

#ser = serial.Serial("/dev/ttyAMA0", 115200)

class AxleModule():
    def runCommand(self, runDirection = 0, brake = False, pwm = 0, lastPwm = 0, lastRunDirection = 1):
        FORWARD_PIN = 17 #GPIO17
        BACKWARD_PIN = 18 #GPIO18
        BRAKE_PIN = 27 #GPIO27
        PWM_PIN = 12 #GPIO12
        PWM_FREQ = 50

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(FORWARD_PIN , GPIO.OUT)
        GPIO.setup(BACKWARD_PIN , GPIO.OUT)
        GPIO.setup(BRAKE_PIN , GPIO.OUT)
        
        pwmHw = HardwarePWM(pwm_channel=0, hz=PWM_FREQ)
        
        if brake:
            GPIO.output(FORWARD_PIN, GPIO.LOW)
            GPIO.output(BACKWARD_PIN, GPIO.LOW)
            GPIO.output(BRAKE_PIN, GPIO.LOW)
            pwmHw.start(0)
            print('brake')
        #empty
        elif runDirection == 0:
            GPIO.output(BRAKE_PIN, GPIO.HIGH)
            GPIO.output(FORWARD_PIN, GPIO.LOW)
            GPIO.output(BACKWARD_PIN, GPIO.LOW)            
            pwmHw.start(0)
            print('empty')
        #forward
        elif runDirection == 1:
            GPIO.output(BRAKE_PIN, GPIO.HIGH)
            GPIO.output(FORWARD_PIN, GPIO.HIGH)
            GPIO.output(BACKWARD_PIN, GPIO.LOW)            
            pwmHw.start(pwm)
            print('forward pwm = ', pwm)
        #backward
        elif runDirection == -1:
            GPIO.output(BRAKE_PIN, GPIO.HIGH)
            GPIO.output(FORWARD_PIN, GPIO.LOW)
            GPIO.output(BACKWARD_PIN, GPIO.HIGH)
            pwmHw.start(pwm)
            print('backward pwm = ', pwm)

            
            
    
#test main
if __name__ == '__main__':


    '''
    AxleModule().runCommand(runDirection = 1)
    input('forward')
    AxleModule().runCommand(runDirection = -1)
    input('backward')
    '''

    
    
    while True:
        for duty in range(28, 35, 1):
            AxleModule().runCommand(runDirection = 1, brake=False, pwm=duty)
            time.sleep(5)
        
    