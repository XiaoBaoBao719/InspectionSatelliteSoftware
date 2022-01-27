#!/bin/bash

clear
echo "THe script starts here"
echo "Hello $USER"
echo "Here are some variables"
COLOR="black"
VALUE=9
echo "$COLOR and $VALUE are here!"

echo "Good bye!"
echo 

python /home/pi/Mask_Handrail/det2_handrail_predictor.py

