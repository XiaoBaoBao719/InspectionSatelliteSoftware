""" Extract BGR pixel vlaues of Handrail

Extracts BGR values of pixels covering the handrail  the for determining lighting environment

Input [type XX, type json]: in-flight captured image, truthBGR_xy_coor.json
Output [n/a]: clear outputData.json and write (to outputData.json )BGR values 
                for known pixel locations (locations are determined 
                post-integration/pre-handoff and stored in truthBGR_xy_coor.json); 
                write output to outputData.json

Author(s): Billy Orion Mazotti

Last Edited: 2/15/22

"""

import json
import cv2
import numpy as np


image_directory = "handrail_calib_1.jpg" #.# caputred image

with open("truthBGR_xy_coor.json", 'r') as f:
    pix_xy = json.load(f)

pix_BGR = []
orig_img = cv2.imread(image_directory) # creates a numpy array object

pix_xy = np.array(pix_xy, dtype=int)
pix_xy = np.ndarray.tolist(pix_xy)
for i in range(len(pix_xy)):
    x_BGR, y_BGR = int(pix_xy[i][0]),int(pix_xy[i][1])
    B,G,R = orig_img[y_BGR,x_BGR]
    pix_BGR.append([B,G,R])

with open("outputData.json", "w") as outfile:
    outfile.write(str(pix_BGR))