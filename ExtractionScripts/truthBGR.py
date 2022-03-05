""" Extract BGR pixel vlaues of Handrail

Extracts BGR values of pixels covering the handrail the for determining lighting environment

Author(s): Billy Orion Mazotti

Last Edited: 2/15/22

"""

import json
import cv2
import numpy as np

def truthBGR(image_directory):
  """ Extracts BGR values of pixels covering the handrail  the for determining 
      lighting environment

      Input: in-flight captured image, truthBGR_xy_coor.json
      Output: clear outputData.json and write (to outputData.json) BGR values 
              for known pixel locations (locations are determined 
              post-integration/pre-handoff and stored in truthBGR_xy_coor.json); 
              write output to outputData.json
  """
  with open("truthBGR_xy_coor.json", 'r') as f:
    pix_xy = json.load(f)

  pix_BGR = []
  orig_img = cv2.imread(image_directory)

  pix_xy = np.array(pix_xy, dtype=int)
  pix_xy = np.ndarray.tolist(pix_xy)
  for i in range(len(pix_xy)):
    x_BGR, y_BGR = int(pix_xy[i][0]),int(pix_xy[i][1])
    B,G,R = orig_img[y_BGR,x_BGR]
    pix_BGR.append([B,G,R])

  with open("outputData.json", "w") as outfile:
    outfile.write(str(pix_BGR))

  pass


# ### INPUT VARIABLES FOR TESTING ###
image_directory = "handrail_single_real.jpg"            #.# caputred image
truthBGR(image_directory)
img_captured = cv2.imread(image_directory)
h, w, channels = img_captured.shape
print(h,w)

with open("truthBGR_xy_coor.json", 'r') as f:
  pix_xy = json.load(f)

pix_xy = np.array(pix_xy, dtype=int)
pix_xy = np.ndarray.tolist(pix_xy)

print(pix_xy)

img_visual = np.zeros((h,w,3), dtype=np.uint8)
for [x,y] in pix_xy:
  B,G,R = img_captured[y, x]
  img_visual[y,x]=[B, G, R]

img_visual = cv2.rectangle(img_visual, (0,0), (49,49),(255,255,255))
print(len(pix_xy))

# cv2.imshow("handrail_visual_of_pix_select.jpg", img_visual)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
cv2.imwrite("handrail_DOWNLINK_VISUAL.jpg", img_visual)



