""" Burnwire Class
@author: Xiao-Bao Bao
@license: GNU Commons
@version 1.0

@brief BUrnwire Python source file 

Uses Raspberry Pi GPIO pins in order to control S/C burnwire configuration and deployment
via the use of Pulse WIdth Modulation (PWM) pin output.
"""

from multiprocessing.connection import wait
from RPIO import PWM
from time import sleep
import RPi.GPIO as GPIO

from FSW.Main import Burn_Wire

# =====================================
# ==         GLOBAL VARS         ==
# =====================================
#PD_POS = 20 # Photodiode GPIO20
#INPUT_PIN = 10
BURN_POS_1 = 12 # GPIO12
BURN_POS_2 = 13 # GPIO13

BURN_CHANNELS = {'channel_1':BURN_POS_1, 'channel_2':BURN_POS_2}

DEF_DUTY_CYCLE = 100 # percentage
DEF_FREQ = 5000 # Hz
DEF_DUR = 5 # seconds

class Burnwire:
    
    burnwireStatus = ""
    #burn_pins = (1, 2)
    burn_pos_1 = GPIO.PWM() # GPIO.PWM([pin], [frequency])
    burn_pos_2 = GPIO.PWM() # pwm.start([duty cycle])
    freq = 500 # kHz
    duty_cyle = 0 # % percentage
    
    def __init__(self, num_burnwires, init_freq=DEF_FREQ, init_duty_cycle=DEF_DUTY_CYCLE):
        print("Initializing Burnwire")
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(BURN_CHANNELS, GPIO.OUT)
        GPIO.output(BURN_CHANNELS, GPIO.LOW)
        #GPIO.input(PD_POS, GPIO.IN)
        
        self.freq = init_freq
        self.duty_cycle = init_duty_cycle
        self.burn_pwm_1 = GPIO.PWM(BURN_CHANNELS['channel_1'], init_freq)
        self.burn_pwm_2 = GPIO.PWM(BURN_CHANNELS['channel_2'], init_freq)
        self.burn_pwm_1.start(init_duty_cycle)
        self.burn_pwm_2.start(init_duty_cycle)
        burnWireStatus = "Burnwire Duty Cycle: %d \n Frequency: %d "(self.duty_cycle, self.freq)
    
    def getBurnwireStatus(self):
        if GPIO.input(BURN_CHANNELS['channel_1']):
            print("Burnwire 1 Active")
        elif GPIO.input(BURN_CHANNELS['channel_2']):
            print ("Burnwire 2 Active")
        else :
            print ("Burnwires Inactive")
        return self.burnwireStatus

    def burn(self, burn_num, duty_cycle, freq, duration):
        """ Calls GPIO pins in order to run current through burn resistor and
            trigger the release mechanism
        Parameters
        ----------
        burn_num: int
            Toggles which burnwire to fire
        
        Returns
        ---------
        Returns True if GPIO pins specified were correctly set to desired values
        via PWM. Otherwise, return False in the event of an anomally.
        """
        self.freq = freq
        self.duty_cycle = duty_cycle
        
        # Starts the burn!
        if '1' in burn_num:
            self.burn_pwm_1.ChangeFrequency(self.freq)
            self.burn_pwm_1.ChangeDutyCycle(self.duty_cycle)
        elif '2' in burn_num:
            self.burn_pwm_2.ChangeFrequency(self.freq)
            self.burn_pwm_2.ChangeDutyCycle(self.duty_cycle)
        else:
            return False
        
        # Hold burn for duration
        sleep(duration)
        
        # Cease burn and cleanup
        GPIO.output(BURN_CHANNELS, GPIO.LOW)
        self.burn_pwm_1.ChangeDutyCycle(0)
        self.burn_pwm_2.ChangeDutyCycle(0)
        self.destroy()

        return True
    
    def destroy(self):
        """ Ensures that burnwire output pins cease running and 
            cleans up memory.
        """
        self.burn_pwm_1.stop()
        self.burn_pwm_2.stop()
        GPIO.cleanup()


test_wire = Burnwire(2, 1000, 100)
print(test_wire.getBurnwireStatus())

wait(5)

test_wire.burn()