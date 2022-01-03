""" Main Flight Software Class
@Author: Xiao-Bao Bao
@License: GNU Commons

This file forms the basis of the InspectionCV primary payload and shall be
invoked at a predetermined schedule buy the flight computer running Debian 11
on an ARMx64 core processor. Primary functions include: 
1. 
2. 
3.

"""

from time import time
import numpy as np
import loguru
from loguru import logger
import Burnwire as bw

burnwire_time_alloc = 20 # mins

def SysDiagnostic():
    pass

def reboot():
    pass

def main():
    start_time = initializeSystem()
    SysDiagnostic()
    if not Burnwire.burnwire() or (time()-start_time < burnwire_time_alloc*60):
        Burnwire.burnwire()

        # TODO: need way of checking the photodiode in this process before
        # ensuring that the flag is set to true; run as parallel process?
    if not SysDiagnostic(): 
        reboot()



if __name__ == "__main__":
    main()