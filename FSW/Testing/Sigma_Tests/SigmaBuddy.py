import sys
import os
from pathlib import Path

for p in sys.path:
    print(p)

#sys.path.append('Testing/Sub_Tests') # Only works with an ABSOLUTE path

script_dir = os.path.dirname(__file__)
curmodule_dir = os.path.join(script_dir, '..', 'Sub_Tests')
sys.path.append(curmodule_dir)

print("Parent dir path is: ", Path(__file__).parent)

import SomeMoreTests as sub

sub.sub_tests()

def summon_sigma():
    print("sigma summon test successful")
    pass

