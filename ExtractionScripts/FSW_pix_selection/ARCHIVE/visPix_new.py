import cv2
import numpy as np
import json

def Dot2Rect(x,y):
  d = 4
  x_min = int(x-d)
  y_min = int(y-d)
  x_max = int(x+d)
  y_max = int(y+d)
  return x_min,y_min,x_max,y_max


img = cv2.imread("handrail_calib_1.jpg")
h_img,w_img,channels = img.shape
c_h_img,c_w_img = h_img/2,w_img/2

RGB_black = (0,0,0)
RGB_blue = (0,0,255)



#IOU box
x_min,y_min = 46,167
x_max,y_max = 1072,304
img = cv2.rectangle(img, (x_min,y_min), (x_max,y_max), (RGB_black), 1)

#Standoff Left
x_min,y_min = 73,304
x_max,y_max = 127,310
img = cv2.rectangle(img, (x_min,y_min), (x_max,y_max), (RGB_black), 1)

#Standoff Right
x_min,y_min = 979,304
x_max,y_max = 1036,310
img = cv2.rectangle(img, (x_min,y_min), (x_max,y_max), (RGB_black), 1)

### Standoff Right ###
x_min,y_min = 979,304
x_max,y_max = 1036,310
img = cv2.rectangle(img, (x_min,y_min), (x_max,y_max), (RGB_black), 1)

pts = np.array([[10,5],[20,30],[70,20],[50,10]], np.int32)
pts = pts.reshape((-1,1,2))
img = cv2.polylines(img,[pts],True,RGB_black,4)

### PLOT INTERVALS BETWEEN 2 POINTS ###
x1,y1 = 50,50
x2,y2 = 500,50
pts = np.array([[x1,y1],[x2,y2]], np.int32)
pts = pts.reshape((-1,1,2))
# print("Poly Shape: ",pts.shape)
img = cv2.polylines(img,[pts],True,RGB_black,4)

num_vert_int = 1                            # number of verital intervals (i.e. y1=y2)
num_pix_per_int = 3                         # number of pixels per interval
lins = np.linspace(x1,x2,num_vert_int+2)    
# print(lins)

### PLOT POLYGON LINES ###
LB_json_file = "dataset.json"

with open(LB_json_file, 'r') as f:
    via_dict = json.load(f)
    
# create empty dictionary to serve as 
x_points = via_dict['ckxxukz4fvvix0z961wi3gd94.jpg690284']['regions']['0']['shape_attributes']['all_points_x']
y_points = via_dict['ckxxukz4fvvix0z961wi3gd94.jpg690284']['regions']['0']['shape_attributes']['all_points_y']

# print(np.array(x_points).shape)
# print("x_points: ",x_points)

# print(np.array(x_points).shape)
# print("y_points: ",y_points)

poly_points = []
for i in range(len(x_points)):
    poly_points.append([x_points[i],y_points[i]])

poly_points = np.array(poly_points, np.int32)
poly_points = poly_points.reshape((-1,1,2))
img = cv2.polylines(img,[poly_points],True,RGB_blue,4)
# print(poly_points)
# print(poly_points.shape)
x1,y1 = 46,167
x2,y2 = 1062,167
x_mid = int(np.average([x1,x2]))
y_mid = int(np.average([y1,y2]))
i = 1
for i in range(poly_points.shape[0]-1):
    x1 = poly_points[i][0][0]
    y1 = poly_points[i][0][1]
    x2 = poly_points[i+1][0][0]
    y2 = poly_points[i+1][0][1]
    x_mid = int(np.average([x1,x2]))
    y_mid = int(np.average([y1,y2]))
    img = cv2.putText (img, str(i), (x_mid,y_mid), cv2.FONT_HERSHEY_SIMPLEX, 0.5, RGB_black, 2)


print("Poly Shape: ",poly_points.shape)

### Display Image ###
scale_percent = 75 # percent of original size
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)
img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
cv2.imshow("handrail.jpg", img)
cv2.waitKey(0)