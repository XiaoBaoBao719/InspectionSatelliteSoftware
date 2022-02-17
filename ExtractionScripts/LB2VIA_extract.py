""" Reformat Labelbox JSON Polygon Coordinates 
    --> [x1,y1], [x2,y2],...
    to format used by inference_Mask.py and pix_selection.py
    --> [x1,x2,...], [y1,y2,...]

Input: LabelBox_josn_directory
Output: Dictionary of detections containing detection bounding 
        box coordinates and detection confidence; have ordered s.t. 
        detections[0] has highest confidence and detections[-1] has 
        lowest confidence 

Author(s): Billy Orion Mazotti

Last Edited: 2/16/22

"""

import json
import urllib.request
import urllib.parse
import cv2              # opencv-python             4.5.3.56                 pypi_0    pypi
import numpy as np      # numpy                     1.21.2           py37h940b05c_0    conda-forge

def extractLB(LB_json_file):
    '''convert LabelBox polygon annotation 
    dataset format to VIA polygon annotation 
    dataset format for Mask R-CNN model training'''

    # load LabelBox json data as dictionary
    with open(LB_json_file, 'r') as f:
        labelbox_dict = json.load(f)
    
    # create empty dictionary to serve as 
    dictionary = {}

    i = 0
    for image in labelbox_dict:
        ID = image['ID']
        print(i)
        print(ID)
        Labeled_Data_url = image['Labeled Data']
        
        # download and save image to local folder from labelbox url
        file_path = ''.join([ID, '.jpg'])
        urllib.request.urlretrieve(Labeled_Data_url, file_path)

        # use image name and iamge size to create VIA naming convention
        img = cv2.imread(file_path)
        h, w, channels = img.shape
        size = h*w
        via_ImageName = ''.join([ID,'.jpg',str(size)])
        via_FileName = ''.join([ID,'.jpg'])
        
        dictionary[via_ImageName] = {}
        dictionary[via_ImageName]["fileref"] = ""
        dictionary[via_ImageName]["size"] = size
        dictionary[via_ImageName]["filename"] = str(via_FileName)
        dictionary[via_ImageName]["base64_img_data"] = ""
        dictionary[via_ImageName]["file_attributes"] = {}
        dictionary[via_ImageName]["regions"] = {}

        
        objects = labelbox_dict[i]['Label']["objects"]
        
        j = 0
        for j in range(len(objects)):
            #print(j)
            feature_i = [objects][0][j]
            print(j, ': is there a polygon? --> ', ('polygon' in feature_i))

            if('polygon' in feature_i):
                dictionary[via_ImageName]["regions"][str(j)] = {}

                # next export destination
                dest = dictionary[via_ImageName]["regions"][str(j)]
                dest["shape_attributes"] = {}
                dest["shape_attributes"]["name"] = "polygon"


                via_x_list = np.array([])
                via_y_list = np.array([])
                polygon = labelbox_dict[i]['Label']["objects"][j]["polygon"]
                handrail_class = labelbox_dict[i]['Label']["objects"][j]["value"]

                # create serperate lists for polygon x and y coordinates
                for z in range(len(polygon)):
                    
                    # print(featureID['polygon'][x]['x'])
                    via_x_list = np.append(via_x_list, np.array(int(polygon[z]['x'])))

                    # print(featureID['polygon'][x]['y'])
                    via_y_list = np.append(via_y_list, np.array(int(polygon[z]['y'])))

                dest["shape_attributes"]["all_points_x"] = via_x_list.tolist()
                dest["shape_attributes"]["all_points_y"] = via_y_list.tolist()
                dest["region_attributes"] = {"title":handrail_class}
                print(handrail_class)
        
        i = i+1
        
    # serializing json 
    json_object = json.dumps(dictionary)
    # json_object = json.dumps(via_dict, indent=4)

    
    # writing to sample.json
    with open("polygonHandrial.json", "w") as outfile:
        outfile.write(json_object)

# input directory of labelbox json file
LabelBox_josn_directory = 'labelboxPolygon.json'

extractLB(LabelBox_josn_directory)