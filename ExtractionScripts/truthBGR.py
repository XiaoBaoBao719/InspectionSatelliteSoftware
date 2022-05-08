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
def Dot2Rect(x,y):  
    """
    Find pixel points around a coordinate of interest 
    for visualizing pixel placement in image
    Input: coordinates of interst
    Output: min and max x,y to define rectangle"""
    d = 5
    x_min = int(x-d)
    y_min = int(y-d)
    x_max = int(x+d)
    y_max = int(y+d)

    return x_min, y_min,x_max,y_max

image_directory = "handrail_single_real.jpg"            #.# caputred image
truthBGR(image_directory)
img_captured = cv2.imread(image_directory)
DATA_GRAYSCALE = True


if DATA_GRAYSCALE:
  img_captured = cv2.cvtColor(img_captured, cv2.COLOR_BGR2GRAY)
  h, w = img_captured.shape
else:
  h, w, channels = img_captured.shape
print(h,w)

with open("truthBGR_xy_coor.json", 'r') as f:
  pix_xy = json.load(f)

pix_xy = np.array(pix_xy, dtype=int)
pix_xy = np.ndarray.tolist(pix_xy)

print(pix_xy)

img_visual = np.zeros((h,w,3), dtype=np.uint8)
for [x,y] in pix_xy:
  if DATA_GRAYSCALE:
    G = img_captured[y, x]
    color = (G)
    img_visual[y,x]= [G]
    print(G)
  else:
    B,G,R = img_captured[y, x]
    color = (B,G,R)
    img_visual[y,x]=[B, G, R]
    print(B,G,R)

  x_c_min,y_c_min,x_c_max,y_c_max = Dot2Rect(x,y)
  
  if DATA_GRAYSCALE:
    img_visual = cv2.rectangle(img_visual, (x_c_min,y_c_min),(x_c_max,y_c_max), (int(G),int(G),int(G)),-1)
  else:
    img_visual = cv2.rectangle(img_visual, (x_c_min,y_c_min),(x_c_max,y_c_max), (int(B),int(G),int(R)),-1)

print(len(pix_xy))

# if DATA_GRAYSCALE:
#   img_visual = cv2.cvtColor(img_visual, cv2.COLOR_BGR2GRAY)

# cv2.imshow("handrail_visual_of_pix_select.jpg", img_visual)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
cv2.imwrite("handrail_DOWNLINK_VISUAL.jpg", img_visual)



