""" Return [OUTPUT] from [INPUT]
[DETAILED DESCRIPTION]

Input: 
--> External: viaPolygon.json [type file], inference_Mask.det_dict [type dict]
--> Internal: iou_threshold [type float], MAX_TP [type int], MAX_FP [type int], MAX_FP_RGB [type int]
Output: 
--> outString: TP_record [type list], FP_record [type list]
--> falseposInspect: FP_RGB_record [type list]


Author(s): Billy Orion Mazotti

Last Edited: 2/16/22

"""

import json
import numpy as np
import inference_Mask

def BB_IOU(boxA, boxB):
    """https://gist.github.com/meyerjo/dd3533edc97c81258898f60d8978eddc"""
    # determine the (x, y)-coordinates of the intersection rectangle
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])

    # compute the area of intersection rectangle
    interArea = abs(max((xB - xA, 0)) * max((yB - yA), 0))
    if interArea == 0:
        return 0
    # compute the area of both the prediction and ground-truth
    # rectangles
    boxAArea = abs((boxA[2] - boxA[0]) * (boxA[3] - boxA[1]))
    boxBArea = abs((boxB[2] - boxB[0]) * (boxB[3] - boxB[1]))

    # compute the intersection over union by taking the intersection
    # area and dividing it by the sum of prediction + ground-truth
    # areas - the interesection area
    iou = interArea / float(boxAArea + boxBArea - interArea)
    # return the intersection over union value
    return iou


# EXTERNAL INPUTS
det_dict = inference_Mask.det_dict    # dictionary of detections from inference_Mask.py
viaPolygon = "viaPolygon.json"    # LB

# INTERNAL INPUTS (DOWNLINK DRIVEN)
iou_threshold = 0.5
MAX_TP = 3      # max number of true postiives to record
MAX_FP = 10      # max number of false postiives to record
MAX_FP_RGB = 3    # max number of flase positives of interest for RGB analysis
CONFIDENCE_SIG_FIGS = 3


if (MAX_FP_RGB>MAX_FP):
  print("Error: number of false positives for RGB analysis (FP_4_RGB)\
 is greater than the max number of recorded false positives (MAX_FP)")


# IMPORT TRUTH BBOX LOCATION FOR IOU CALCS
with open(viaPolygon, 'r') as f:
    polyDict = json.load(f)

img_file_name = list(polyDict.keys())[0]
x_points = polyDict[img_file_name]['regions']['0']['shape_attributes']\
['all_points_x']
y_points = polyDict[img_file_name]['regions']['0']['shape_attributes']\
['all_points_y']

x_min = min(x_points)
x_max = max(x_points)
y_min = min(y_points)
y_max = max(y_points)
boxA = [x_min,y_min,x_max,y_max]


# DETERMINE WHICH DETECTIONS ARE TRUE POSITIVE/FALSE POSITIVES
for detection in det_dict:
  boxB = det_dict[detection]["bbox"]
  det_dict[detection]["IOU"] = round(BB_IOU(boxA,boxB),3)
  if(det_dict[detection]["IOU"] >= iou_threshold):
    det_dict[detection]["TP?"] = True
  else:
    det_dict[detection]["TP?"] = False


# #################################
# # TABULATED DETECTION RESULTS
# from tabulate import tabulate

# spc1 = "                 "
# spc2 = "              "

# # Includes IOU and TP?
# headers = ["Detection","Bbox"+spc1+spc1+spc1+"Conf"+ spc2+"IOU"+spc2+"TP?"] 

# table = tabulate(det_dict.items(),headers = headers)
# print(table)

# #################################


# PUT TRUE/FALSE POSITIVES INTO SEPERATE DICTIONARIES
TP_dict_mix = {}      # true positive dicitonary (unsorted)
FP_dict = {}          # false positive dictionary (naturally sorted by Mask)

countTP = 0           
countFP = 0
for detection in det_dict:

  if(det_dict[detection]["IOU"] >= iou_threshold):
    TP_dict_mix.update({countTP: det_dict[detection]})
    countTP = countTP + 1
  else:
    FP_dict.update({countFP: det_dict[detection]})
    countFP = countFP + 1

# print("TP_dict mixed: ", TP_dict_mix)
# print("FP_dict: ", FP_dict)


# SORT TP_dict BY IOU VALUE FROM HIGHEST TO LOWEST
TP_dict_keys_mix = list(TP_dict_mix.keys())
TP_dict_keys_sort = sorted(TP_dict_keys_mix, key=lambda x: 
                           (TP_dict_mix[x]['IOU']), reverse=True)

TP_dict_sort = {}
for i in range(len(TP_dict_keys_sort)):
	TP_dict_sort.update({i:TP_dict_mix[TP_dict_keys_sort[i]]})
 
# print("TP_dict sorted Dict: ", TP_dict_sort)


# EXPORT DETECTIONS (LIMITED BY MAX_TP, MAX_FP)
TP_record = []
FP_record = []
FP_RGB_record = []

TP_exist = len(TP_dict_sort)
FP_exist = len(FP_dict)

if MAX_TP <= TP_exist:
  TP_2_downlink = MAX_TP
else: 
  TP_2_downlink = TP_exist

if MAX_FP <= FP_exist:
  FP_2_downlink = MAX_FP
else: 
  FP_2_downlink = FP_exist

if MAX_FP_RGB <= FP_exist:
  FP_4_RGB = MAX_FP_RGB
else:
  FP_4_RGB = FP_exist

# returns MAX_TP or TP_exist many FPs for falseposInspect.py
for detection in range(TP_2_downlink):
  TP_record.append(TP_dict_sort[detection])

# returns MAX_FP or FP_exist many FPs for falseposInspect.py
for detection in range(FP_2_downlink):
  FP_record.append(FP_dict[detection])

# returns MAX_FP_RGB or FP_exist many FPs for falseposInspect.py
for detection in range(FP_4_RGB):
  FP_RGB_record.append(FP_dict[detection])

print("TP downlinked: ", TP_record)
print("FP downlinked: ",FP_record)
print("FP downlinked: ",FP_RGB_record)


with open("outputData.json", 'r') as f:
    outputData = json.load(f)

for i in range(len(TP_record)):
  outputData.append(outputData, TP_record[i]['bbox'])
  outputData.append(outputData, round(TP_record[i]['conf'],CONFIDENCE_SIG_FIGS))

for i in range(len(FP_record)):
  outputData.append(outputData, FP_record[i]['bbox'])
  outputData.append(outputData, round(FP_record[i]['conf'],CONFIDENCE_SIG_FIGS))

with open("outputData.json", "w") as outfile:
  outfile.write(str(outputData))