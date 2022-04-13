# Timer class that helps the FSW keep track of time on the device

import time

class TimeError(Exception):
    """ Class to handle exceptions in the timer class"""

class Timer:
    def __init__(self) -> None:
        self.start_time = None
    
    def start(self):
        """ Starts a new timer"""
        if self.start_time is not None:
            raise TimeError(f"Timer is not running, please instantiate the timer")

        self.start_time = time.perf_counter()

    def stop(self): 
        """Cease the timer and report how much time has elapssed"""
        if self.start_time is None:
            raise TimeError(f"Timer was not initialized, please start the timer first!")

        elapsed_time =  time.perf_counter() - self.start_time
        self.start_time = 0
        print(f"Stopping the timer \n Total elapsed time passed is: {elapsed_time:0.4f} seconds")

        return elapsed_time