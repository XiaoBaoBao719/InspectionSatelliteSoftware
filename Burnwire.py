""" Burnwire Class
@author: Xiao-Bao Bao
@license: GNU Commons
@version 1.0

@brief BUrnwire Python source file 

Uses Raspberry Pi GPIO pins in order to control S/C burnwire configuration and deployment
via the use of Pulse WIdth Modulation (PWM) pin output.
"""
import RPi.GPIO as GPIO
from multiprocessing.connection import wait
from time import sleep
# from FSW.Main import Burn_Wire

# =====================================
# ==         GLOBAL VARS         ==
# =====================================
#PD_POS = 20 # Photodiode GPIO20 
#INPUT_PIN = 10
BURN_EN    =  6 # GPIO 6
BURN_PIN_1 = 12 # GPIO12
BURN_PIN_2 = 13 # GPIO13
BURN_CHANNELS = (BURN_PIN_1, BURN_PIN_2)

DEF_DUTY_CYCLE = 100 # percentage
DEF_FREQ = 500 # khz
DEF_DUR =  10 # seconds

class Burnwire:
    
    burnwireStatus = ""
    #burn_pins = (1, 2)
    # burn_pwm_1 = None
    # burn_pwm_2 = None
    burn_status_1 = False
    burn_status_2 = False
    # freq = 500 # kHz
    duty_cyle = 100 # % percentage
    
    """
    def __init__(self, num_burnwires=2, init_freq=DEF_FREQ, init_duty_cycle=DEF_DUTY_CYCLE):
        print("Initializing Burnwire")
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(BURN_CHANNELS, GPIO.OUT)
        GPIO.output(BURN_CHANNELS, GPIO.LOW)
        #GPIO.input(PD_POS, GPIO.IN)
        
        self.burn_pwm_1 = GPIO.PWM(BURN_POS_1, init_freq) # GPIO.PWM([pin], [frequency])
        self.burn_pwm_2 = GPIO.PWM(BURN_POS_2, init_freq) # pwm.start([duty cycle])
        
        self.freq = init_freq
        self.duty_cycle = init_duty_cycle
        # self.burn_pwm_1 = GPIO.PWM(BURN_CHANNELS[0], init_freq)
        # self.burn_pwm_2 = GPIO.PWM(BURN_CHANNELS[1], init_freq)
        self.burn_pwm_1.start(init_duty_cycle)
        self.burn_pwm_2.start(init_duty_cycle)
        burnWireStatus = "Burnwire Duty Cycle: %d \n Frequency: %d ", (self.duty_cycle, self.freq)
    """
    def __init__(self, burnwires):
        print("Initializing burnwires")
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(BURN_EN, GPIO.OUT)
        GPIO.setup(BURN_CHANNELS, GPIO.OUT)
        GPIO.output(BURN_EN, GPIO.LOW)
        GPIO.output(BURN_CHANNELS, GPIO.LOW)
        self.burn_status_1 = False
        self.burn_status_2 = False
    
    def getBurnwireStatus(self):
        if self.burn_status_1:
            print("Burnwire 1 Active")
        if self.burn_status_2:
            print ("Burnwire 2 Active")
        elif not self.burn_status_1 and self.burn_status_2:
            print ("Burnwires Inactive")
        return 

    def burn(self, burn_channels, duty_cycle=DEF_DUTY_CYCLE, freq=DEF_FREQ, duration=DEF_DUR):
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
        # self.freq = freq
        # self.duty_cycle = duty_cycle
        # print(burn_channels)
        # Starts the burn!
        if burn_channels == None or len(burn_channels) == 0:
            print("Expected burn channels!")
            return False
        
        for channel in burn_channels:
            if channel == 1:
                # self.burn_pwm_1.ChangeFrequency(self.freq)
                # self.burn_pwm_1.ChangeDutyCycle(self.duty_cycle)
                GPIO.output(BURN_EN,    GPIO.HIGH)
                GPIO.output(BURN_PIN_1, GPIO.HIGH)
                self.burn_status_1 = True
                
            if channel == 2:
                # self.burn_pwm_2.ChangeFrequency(self.freq)
                # self.burn_pwm_2.ChangeDutyCycle(self.duty_cycle)
                GPIO.output(BURN_EN,    GPIO.HIGH)
                GPIO.output(BURN_PIN_2, GPIO.HIGH)
                self.burn_status_2 = True
            sleep(0.2)
        # Hold burn for duration
        self.getBurnwireStatus()
        print("Burning!")
        sleep(duration)
        
        # Cease burn and cleanup
        print("Finished burning!")
        
        burn_status_1 = False
        burn_status_2 = False
        # self.burn_pwm_1.ChangeDutyCycle(0)
        # self.burn_pwm_2.ChangeDutyCycle(0)
        self.getBurnwireStatus()
        self.destroy()
        return True
    
    def destroy(self):
        """ Ensures that burnwire output pins cease running and 
            cleans up memory.
        """
        # self.burn_pwm_1.stop()
        # self.burn_pwm_2.stop()
        print("Cleaning up burnwire...")
        GPIO.output(BURN_CHANNELS, GPIO.LOW)
        GPIO.output(BURN_EN, GPIO.LOW)
        GPIO.cleanup()

# Debugging Area
"""
test_wire = Burnwire(2)
test_wire.getBurnwireStatus()
sleep(2)
burn_channels = [1, 2]
burn_result = test_wire.burn(burn_channels, duration=5)
if burn_result:
    print("Burn successful!")
else:
    print("Burn attempt failed!")
"""

