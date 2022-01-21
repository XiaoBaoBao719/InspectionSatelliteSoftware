import sys
sys.path.append('/home/xiaobao/InspectionSatCV/FSW/')

import TestBuddy
import Sub_Tests.SomeMoreTests as sub

TestBuddy.one_test() # Adjcent dir

sub.sub_tests() # Testing sub directories

