""" Select pixel coordinates for BGR extraction of Handrail

Selects and creates list of [x,y] coordinaetes within field of view

Input: post-assembly/pre-handoff image, viaPolygon.json
Output: write [x,y] coordinates list to truthBGR_cy_coor.json

Author(s): Billy Orion Mazotti

Last Edited: 2/15/22

"""

import cv2
import numpy as np
import json

def DotProduct(a,b):
    """
    Find the Dot Product between vectors a and b
    Input: vector a, vector b
    Output: angle ans"""
    ans = np.arccos((np.dot(a,b) / (np.linalg.norm(a)*np.linalg.norm(b)))) *180/np.pi
    
    return ans

def Dot2Rect(x,y):  
    """
    Find pixel points around a coordinate of interest 
    for visualizing pixel placement in image
    Input: coordinates of interst
    Output: min and max x,y to define rectangle"""
    d = 1
    x_min = int(x-d)
    y_min = int(y-d)
    x_max = int(x+d)
    y_max = int(y+d)
    
    return x_min,y_min,x_max,y_max

def rmvSides(list):
    """remove the first and last elements in a list
    """
    list = np.delete(list, 0)
    list = np.delete(list, -1)

    return list

def Matrix2List(x,y):
    """ given seperate x coordinate list and y coordinate list of equal length
    Input: x [type list], y [type list]
    """
    c = []
    for i in range(len(x)):
        for j in range(len(y)):
            c.append([x[i],y[j]])
    
    return c

def AppendElements(new, old):
    """add elements from list "old" to list "new"
    """
    for i in range(len(old)):
        new.append(old[i])
    
    return new


############### COLORS FOR VISUALS ###############
BGR_black = (0,0,0)
BGR_red = (0,0,255)
BGR_green = (0,255,0)
BGR_blue = (255,0,0)
BGR_orange = (0, 165, 255)
BGR_purple = (221,160,221)
BGR_gray = (220,220,220)


############### CALIBRATION IMPORTS ###############
LB_json_file = "viaPolygon_single_real.json"                 #.# json created from LB2VIA_extract
image_directory = "handrail_single_real.jpg"           #.# pre-flight caputred image


############### CALIBRATION PARAMETERS ###############
""" Adjust the following parameters for pixel selection and 
    visualization post-assembly/pre-handoff

    Will be adjusted to accomodate downlink limits
"""
###### EDGE PIXELS ######
SHOW_POLYLINES = False
SHOW_EDGE_PIXELS = True
# number of pixels
num_intervals = 0
num_pix_per_interval = 5                # at first, set to 1
pix2pix_dist = 10                       # at first, set to 1


###### SURFACE PIXELS ######
SHOW_ENTIRE_HANDRAIL_BBOX = False

### RAIL ###
# bbox location
SHOW_RAIL_BBOX = False
SHOW_RAIL_PIX =True
c_l_rail, c_r_rail = -0.07, 1.06
c_t_rail, c_b_rail = 0.02, 0.47
# number of pixels
numRow_rail = 5
numCol_rail = 10

### LEFT STANDOFFF ###
# bbox location
SHOW_LEFT_STANDOFF_BBOX = False
SHOW_LEFT_STANDOFF_PIX =True
c_l_L, c_r_L = -0.025, 0.2
c_t_L, c_b_L = 0.45, 1.0
# number of pixels
numRowL = 7
numColL = 7

### RIGHT STANDOFFF ###
# bbox location
SHOW_RIGHT_STANDOFF_BBOX = False
SHOW_RIGHT_STANDOFF_PIX =True
c_l_R, c_r_R = 0.78, 1.01
c_t_R, c_b_R = 0.45, 1.0
# number of pixels
numRowR = 7
numColR = 7


############### OUPUT VARIABLES ###############
pix_xy = []
pix_xy_edge = []
pix_xy_surface = []


##################################################
img = cv2.imread(image_directory)
h_img,w_img,channels = img.shape

# plot polygon lines
with open(LB_json_file, 'r') as f:
    polyDict = json.load(f)
    
img_file_name = list(polyDict.keys())[0]
x_points = polyDict[img_file_name]['regions']['0']['shape_attributes']['all_points_x']
y_points = polyDict[img_file_name]['regions']['0']['shape_attributes']['all_points_y']

poly_points = []
for i in range(len(x_points)):
    poly_points.append([x_points[i],y_points[i]])

poly_points = np.array(poly_points, np.int32)
poly_points = poly_points.reshape((-1,1,2))
if SHOW_POLYLINES:
    img = cv2.polylines(img,[poly_points],True,BGR_red,4)
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
    if SHOW_POLYLINES:
        img = cv2.putText (img, str(i), (x_mid,y_mid), cv2.FONT_HERSHEY_SIMPLEX, 0.5, BGR_black, 2)


print("Poly Shape: ",poly_points.shape)

### Display Image ###
scale_percent = 100 # percent of original size
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)
img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)


# number of pixels on each side of the polygon outline
num_pix_per_arm = int((num_pix_per_interval-1)/2)

# select lines of interst (LOI) where you want an interval
LOI = np.linspace(0,len(poly_points)-1,num_intervals, dtype=int)       


for i in range(len(LOI)):
    # find endpoints of each line
    x1 = poly_points[LOI[i]][0][0]
    y1 = poly_points[LOI[i]][0][1]
    
    if LOI[i] == len(poly_points)-1:
        x2 = poly_points[LOI[0]][0][0]
        y2 = poly_points[LOI[0]][0][1]
    else:
        x2 = poly_points[LOI[i]+1][0][0]
        y2 = poly_points[LOI[i]+1][0][1]

    vx,vy = x2-x1,y2-y1

    # determine line orientation
    quad1,quad2,quad3,quad4 = False,False,False,False
    if(vx>=0 and vy<=0):
        quad1 = True
        # print("QUAD 1")
    elif(vx>=0 and vy>=0):
        quad4 = True
        # print("QUAD 2")
    elif(vx<=0 and vy>=0):
        quad3 = True
        # print("QUAD 3")
    elif(vx<=0 and vy<=0):
        quad2 = True
        # print("QUAD 4")

    vec_0 = [1,0]               # vector corresponding to 0 degrees
    vec_LOI = [vx,vy]           # vector of LOI 

    # find the angle THETA1 of LOI wrt vec_0
    # find the angle THETA2 of a line tangent to
    if quad1:
        theta1 = DotProduct(vec_0,vec_LOI)
        theta2 = -theta1 + 90
    elif quad4:
        theta1 = DotProduct(vec_0,vec_LOI)
        theta2 = -theta1 + 90
    elif quad3:
        theta1 = DotProduct(vec_0,vec_LOI) 
        theta2 = theta1 - 90
    elif quad2:
        theta1 = DotProduct(vec_0,vec_LOI) 
        theta2 = theta1 - 90

    # print("t1: ", theta1)
    # print("t2: ", theta2)
    theta2 = theta2*np.pi/180

    # locate the interval at the center of the line
    xc,yc = (x1+x2)/2, (y1+y2)/2
    
    xc_c,yc_c = [],[]
    xc_cc,yc_cc = [],[]
    for i in range(num_pix_per_arm):

        # theta2 = 90*np.pi/180
        if(quad1):
            # counter clockwise pixels extending from tangent line
            xc_c.append(xc - (i+1)*pix2pix_dist*np.cos(theta2))
            yc_c.append(yc - (i+1)*pix2pix_dist*np.sin(theta2))

            # clockwise pixels extending from tangent line
            xc_cc.append(xc + (i+1)*pix2pix_dist*np.cos(theta2))
            yc_cc.append(yc + (i+1)*pix2pix_dist*np.sin(theta2))
        elif(quad4):
            # counter clockwise pixels extending from tangent line
            xc_c.append(xc + (i+1)*pix2pix_dist*np.cos(theta2))
            yc_c.append(yc - (i+1)*pix2pix_dist*np.sin(theta2))

            # clockwise pixels extending from tangent line
            xc_cc.append(xc - (i+1)*pix2pix_dist*np.cos(theta2))
            yc_cc.append(yc + (i+1)*pix2pix_dist*np.sin(theta2))
        elif(quad3):
            # counter clockwise pixels extending from tangent line
            xc_c.append(xc + (i+1)*pix2pix_dist*np.cos(theta2))
            yc_c.append(yc + (i+1)*pix2pix_dist*np.sin(theta2))

            # clockwise pixels extending from tangent line
            xc_cc.append(xc - (i+1)*pix2pix_dist*np.cos(theta2))
            yc_cc.append(yc - (i+1)*pix2pix_dist*np.sin(theta2))
        elif(quad2):
            # counter clockwise pixels extending from tangent line
            xc_c.append(xc - (i+1)*pix2pix_dist*np.cos(theta2))
            yc_c.append(yc + (i+1)*pix2pix_dist*np.sin(theta2))

            # clockwise pixels extending from tangent line
            xc_cc.append(xc + (i+1)*pix2pix_dist*np.cos(theta2))
            yc_cc.append(yc - (i+1)*pix2pix_dist*np.sin(theta2))


    # print("pseudo clockwise [xc_c,yc_c]: ", [xc_c,yc_c])
    # print("pseudo counterclockwise [xc_cc,yc_cc]: ", [xc_cc,yc_cc])

    #interweve  x,y coordinates s.t. (x1,y1)...(xn,yn)
    xy_c = []
    xy_cc = []
    for i in range(len(xc_c)):
        
        xy_c.append([xc_c[i],yc_c[i]])
        xy_cc.append([xc_cc[i],yc_cc[i]])

    # print("pseudo clockwise interwoven [xc_c[i],yc_c[i]]: ", xy_c)
    # print("pseudo counterclockwise interwoven [xc_cc[i],yc_cc[i]]: ", xy_cc)
    xy_c.reverse()

    xy_interval = []
    for i in range(num_pix_per_arm):
        xy_interval.append(xy_c[i])

    xy_interval.append([xc,yc])

    for i in range(num_pix_per_arm):
        xy_interval.append(xy_cc[i])


    pts_int = np.array([xy_interval[0],xy_interval[-1]], np.int32)
    pts_int = pts_int.reshape((-1,1,2))
    # print("Poly Shape: ",pts.shape)
    if SHOW_EDGE_PIXELS:
        img = cv2.polylines(img,[pts_int],True,BGR_green,4)

    idx = 0
    for i in xy_interval:
        idx = idx + 1
        temp_var = idx%num_pix_per_interval
        x_c_min,y_c_min,x_c_max,y_c_max = Dot2Rect(i[0],i[1])

        if(temp_var==1):
            temp_color = BGR_orange
        elif (temp_var==0):
            temp_color = BGR_purple
        else:
            temp_color = BGR_blue
        if SHOW_EDGE_PIXELS:
            img = cv2.rectangle(img, (x_c_min,y_c_min), (x_c_max,y_c_max), temp_color, -1)

    pix_xy_edge = AppendElements(pix_xy_edge, xy_interval)


# Starting point
xCoorList = []
yCoorList = []

for i in range(poly_points.shape[0]):
    xCoorList.append(poly_points[i][0][0])
    yCoorList.append(poly_points[i][0][1])


# ENTIRE HANDRAIL BBOX
x_min, x_max = min(xCoorList), max(xCoorList)
y_min, y_max = min(yCoorList), max(yCoorList)
if SHOW_ENTIRE_HANDRAIL_BBOX:
    img = cv2.rectangle(img, (x_min,y_min), (x_max,y_max), (BGR_black), 1)    # box around entire handrail


# RAIL BBOX
x_min_rail, x_max_rail = int(x_min + c_l_rail*(x_max-x_min)), int(x_min + c_r_rail*(x_max-x_min))
y_min_rail, y_max_rail = int(y_min + c_t_rail*(y_max-y_min)), int(y_min + c_b_rail*(y_max-y_min))
if SHOW_RAIL_BBOX:
    img = cv2.rectangle(img, (x_min_rail,y_min_rail), (x_max_rail,y_max_rail), (BGR_black), 1)  # box around rail

# RAIL PIX
xCoorRail = np.linspace(x_min_rail,x_max_rail, numCol_rail+2)
yCoorRail = np.linspace(y_min_rail,y_max_rail, numRow_rail+2)
xCoorRail = rmvSides(xCoorRail)
yCoorRail = rmvSides(yCoorRail)

pix_xy_surface_rail = Matrix2List(xCoorRail,yCoorRail)

for i in range(len(pix_xy_surface_rail)):
    x_c_min,y_c_min,x_c_max,y_c_max = Dot2Rect(pix_xy_surface_rail[i][0],pix_xy_surface_rail[i][1])
    if SHOW_RAIL_PIX:
            img = cv2.rectangle(img, (x_c_min,y_c_min), (x_c_max,y_c_max), BGR_red, -1)


# LEFT STANDOFF BBOX
x_min_stanL, x_max_stanL = int(x_min + c_l_L*(x_max-x_min)), int(x_min + c_r_L*(x_max-x_min))
y_min_stanL, y_max_stanL = int(y_min + c_t_L*(y_max-y_min)), int(y_min + c_b_L*(y_max-y_min))
if SHOW_LEFT_STANDOFF_BBOX:
    img = cv2.rectangle(img, (x_min_stanL,y_min_stanL), (x_max_stanL,y_max_stanL), (BGR_black), 1)  # box around left standoff

# LEFT STANDOFF PIX
xCoor_stanL = np.linspace(x_min_stanL,x_max_stanL, numColL+2)
yCoor_stanL = np.linspace(y_min_stanL,y_max_stanL, numRowL+2)
xCoor_stanL = rmvSides(xCoor_stanL)
yCoor_stanL = rmvSides(yCoor_stanL)

pix_xy_surface_L = Matrix2List(xCoor_stanL,yCoor_stanL)

for i in range(len(pix_xy_surface_L)):
    x_c_min,y_c_min,x_c_max,y_c_max = Dot2Rect(pix_xy_surface_L[i][0],pix_xy_surface_L[i][1])
    if SHOW_LEFT_STANDOFF_PIX:
            img = cv2.rectangle(img, (x_c_min,y_c_min), (x_c_max,y_c_max), BGR_red, -1)


# RIGHT STANDOFF BBOX
x_min_stanR, x_max_stanR = int(x_min + c_l_R*(x_max-x_min)), int(x_min + c_r_R*(x_max-x_min))
y_min_stanR, y_max_stanR = int(y_min + c_t_R*(y_max-y_min)), int(y_min + c_b_R*(y_max-y_min))
if SHOW_RIGHT_STANDOFF_BBOX:
    img = cv2.rectangle(img, (x_min_stanR,y_min_stanR), (x_max_stanR,y_max_stanR), (BGR_black), 1)  # box around right standoff

# RIGHT STANDOFF PIX
xCoor_stanR = np.linspace(x_min_stanR,x_max_stanR, numColR+2)
yCoor_stanR = np.linspace(y_min_stanR,y_max_stanR, numRowR+2)
xCoor_stanR = rmvSides(xCoor_stanR)
yCoor_stanR = rmvSides(yCoor_stanR)

pix_xy_surface_R = Matrix2List(xCoor_stanR,yCoor_stanR)

for i in range(len(pix_xy_surface_R)):
    x_c_min,y_c_min,x_c_max,y_c_max = Dot2Rect(pix_xy_surface_R[i][0],pix_xy_surface_R[i][1])
    if SHOW_RIGHT_STANDOFF_PIX:
            img = cv2.rectangle(img, (x_c_min,y_c_min), (x_c_max,y_c_max), BGR_red, -1)

pix_xy_surface = AppendElements(pix_xy_surface,pix_xy_surface_rail)
pix_xy_surface = AppendElements(pix_xy_surface,pix_xy_surface_L)
pix_xy_surface = AppendElements(pix_xy_surface,pix_xy_surface_R)


pix_xy = AppendElements(pix_xy, pix_xy_edge)
pix_xy = AppendElements(pix_xy, pix_xy_surface)

print("minimum [x,y]: ", min(pix_xy))
print("maximum [x,y]: ", max(pix_xy))
print("Image [Width, Height]: ", [w_img,h_img])

for [x,y] in pix_xy:
    if (x>=w_img or x<0 or y>=h_img or y<0):
        error_FOV = "Pixel Selection Has Exceeded Field of View"
        raise ValueError(error_FOV)

print("Number of Edge Pixels: ", len(pix_xy_edge))
print("Number of Surface Pixels: ", len(pix_xy_surface))
print("Total Number of Pixels: ", len(pix_xy))

with open("truthBGR_xy_coor.json", "w") as outfile:
    outfile.write(str(pix_xy))


# cv2.imshow("handrail_visual_of_pix_select.jpg", img)             # show interval of interest on edited image
# cv2.waitKey(0)
# cv2.destroyAllWindows()
cv2.imwrite("handrail_visual_of_pix_select.jpg", img)

