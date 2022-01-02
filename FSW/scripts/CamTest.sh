#!/bin/bash

echo "Starting Pi Cam Test Process"
echo 
echo "Snapping a Pic, say cheese!"

libcamera-jpeg -o handrail-input.jpg -t 5000 --width 800 --height 600

echo "Mooving to the main dir"
echo
cd /home/pi
echo "Creating a new directory for holding handrail data"
echo
mkdir HandrailData 
cd HandrailData
mkdir observations
cd /home/pi/HandrailData 
mkdir predictions
cd /home/pi

echo "Mooving the handrail observation into the handrail data folder"
mv /home/pi/scripts/handrail-input.jpg /home/pi/HandrailData/observations
echo "Running inference on the pic we just snapped!"
echo

cd /home/pi/Mask_Handrail
python det2_handrail_predictor.py 

echo "Mooving into the hand rail data and moving the visual output in there"
cd /home/pi
mv /home/pi/handrail-output.jpg /home/pi/HandrailData/predictions
#rm /home/pi/handrail-output.jpg

echo "Done writing! Here is the console back :)"
echo



