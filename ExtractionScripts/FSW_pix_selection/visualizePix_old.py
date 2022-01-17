import cv2
import numpy as np

def Dot2Rect(x,y):
  d = 10
  x_min = int(x-d)
  y_min = int(y-d)
  x_max = int(x+d)
  y_max = int(y+d)
  return x_min,y_min,x_max,y_max

handrail = cv2.imread("handrail_calib_1.jpg")

h,w,channels = handrail.shape
print(h)
print(w)
print(channels)

RGB_black = (0,0,0)
RGB_blue = (0,0,255)
RGB_red = (255,0,0)

# handrail truth bbox
h_h_c = h/2     # center height/row
h_w_c = w/2     # center width/column

h_h = (300/623)*h     # [pixels] handrail height
h_w = (1080/1108)*w    # [pixels] handrial width

h_h_off = (7/623)*h       # height/row offset
h_w_off = (-3/1108)*w        # width/column offset

x_min,y_min = int(h_w_c-h_w/2+h_w_off), int(h_h_c-h_h/2+h_h_off)
x_max,y_max = int(h_w_c+h_w/2+h_w_off),int(h_h_c+h_h/2+h_h_off)

# rail bbox
r_h_c = h/2      # center height/row
r_w_c = w/2                   # center width/column

r_h = (150/623)*h     # [pixels] rail height
r_w = (950/1108)*w     # [pixels] rail width

r_h_off = h_h_off - (75/623)*h      # height/row offset
r_w_off = h_w_off       # width/column offset

r_x_min,r_y_min = int(r_w_c - r_w/2 + r_w_off), int(r_h_c - r_h/2 + r_h_off)
r_x_max,r_y_max = int(r_w_c + r_w/2 + r_w_off),int(r_h_c + r_h/2 + r_h_off)


x_c = (x_min+x_max)/2
y_c = (y_min+y_max)/2



# PRINT BOXES
handrail = cv2.rectangle(handrail, (r_x_min,r_y_min), (r_x_max,r_y_max), (RGB_black), 1)



#SURFACE PIXELS
#S.1 RAIL
handrail = cv2.rectangle(handrail, (x_min,y_min), (x_max,y_max), (RGB_black), 1)

s_r_bbox = np.array([[1, 2, 3]]).T


#S.2 STANDOFFS
x_c_min,y_c_min,x_c_max,y_c_max = Dot2Rect(x_c,y_c)
handrail = cv2.rectangle(handrail, (x_c_min,y_c_min), (x_c_max,y_c_max), (RGB_black), -1)


# Display Image
scale_percent = 35 # percent of original size
width = int(handrail.shape[1] * scale_percent / 100)
height = int(handrail.shape[0] * scale_percent / 100)
dim = (width, height)
handrail = cv2.resize(handrail, dim, interpolation = cv2.INTER_AREA)

cv2.imshow("handrail.jpg", handrail)
cv2.waitKey(0)



