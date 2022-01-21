from RPIO import PWM
from time import sleep
import RPi.GPIO as GPIO

# =====================================
# ==         GLOBAL VARS         ==
# =====================================
PD_POS = 20 # Photodiode GPIO20
#INPUT_PIN = 10
BURN_POS_1 = 12 # GPIO12
BURN_POS_2 = 13 # GPIO13

BURN_CHANNELS = {'channel_1':BURN_POS_1, 'channel_2':BURN_POS_2}

class Burnwire:
    
    burnwireStatus = ""
    #burn_pins = (1, 2)
    burn_pos_1 = GPIO.PWM() # GPIO.PWM([pin], [frequency])
    burn_pos_2 = GPIO.PWM() # pwm.start([duty cycle])
    freq = 500 # kHz
    duty_cyle = 0 # % percentage
    
    def __init__(self, num_burnwires, init_freq, init_duty_cycle):
        print("Initializing Burnwire")
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(BURN_CHANNELS, GPIO.OUT)
        GPIO.output(BURN_CHANNELS, GPIO.LOW)
        GPIO.input(PD_POS, GPIO.IN)
        
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
        val: float
            Contains the digital photodiode readings
        Returns
        ---------
        pass
        """
        self.freq = freq
        self.duty_cycle = duty_cycle
        
        # Starts the burn!
        if '1' in burn_num:
            self.burn_pwm_1.ChangeFrequency(self.freq)
            self.burn_pwm_1.ChangeDutyCycle(self.duty_cycle)
        elif '2' in burn_num:
            self.burn_pwm_2.ChangeFrequency(self.freq)
            self.burn_pwm_2ChangeDutyCycle(self.duty_cycle)
        else:
            return False
        
        # Hold burn for duration
        sleep(duration)
        
        # Cease burn and cleanup
        GPIO.output(BURN_CHANNELS, GPIO.LOW)
        self.burn_pwm_1.ChangeDutyCycle(0)
        self.burn_pwm_2.ChangeDutyCycle(0)
        self.destroy()
        
        # Immediately perform burnwire deploy for 5 seconds
        #GPIO.output(BURNWIRE_PINS, GPIO.HIGH)
        #prtStatus = GPIO.output(BURNWIRE_PINS, not GPIO.input(BURNWIRE_PINS))
        #time.sleep(5) # Wait 5 seconds
        # Shutdown burnwire!
        #GPIO.output(BURNWIRE_PINS, GPIO.LOW)
        #prtStatus += GPIO.output(BURNWIRE_PINS, not GPIO.input(BURNWIRE_PINS)) # Report to sys diagnostic
        #time.sleep(10) # Wait 10 seconds
        #pass
        return True
    
    def destroy(self):
        self.burn_pwm_1.stop()
        self.burn_pwm_2.stop()
        GPIO.cleanup()


# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(BURN_PIN_1, GPIO.OUT, initial = GPIO.LOW)
# GPIO.setup(BURN_PIN_2, GPIO.OUT, initial = GPIO.LOW)
# GPIO.setup(INPUT_PIN, GPIO.IN)
# GPIO.setup(PHOTODIODE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
# 
# # Immediately perform burnwire deploy for 5 seconds
# GPIO.output(burnChanList, GPIO.HIGH)
# prtStatus = GPIO.output(burnChanList, not GPIO.input(burnChanList))
# time.sleep(5) # Wait 5 seconds
# # Shutdown burnwire!
# GPIO.output(burnChanList, GPIO.LOW)
# prtStatus += GPIO.output(burnChanList, not GPIO.input(burnChanList)) # Report to sys diagnostic
# time.sleep(2) # Wait 2 seconds
# 
# if burnwireNotDeployed():
#     prtStatus += "Burnwire not deployed, trying again"
#     # Initialize timer to zero, run for 20 mins
# 
# 
#     # Check if burnwire deploy successful 
#     while GPIO.input(PHOTODIODE_PIN) == GPIO.low and timer < 20:
#         # Tell the Main class to not run the Camera Science packet
#         prtStatus += "Waiting for confirmation of successful burnwire"
# GPIO.cleanup()
