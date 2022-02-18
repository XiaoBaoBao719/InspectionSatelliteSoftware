""" Pass Image into Mask R-CNN Model

Runs Mask model and returns detection data for further analysis

Author(s): Billy Orion Mazotti

Last Edited: 2/17/22

"""


# imports
# Functions that need library: get_cfg(), DefaultPredictor(), cv2.imread()

import os
from detectron2 import model_zoo
from detectron2.config import get_cfg
from detectron2.engine import DefaultPredictor
import cv2

def Inference_Mask(im, inference_threshold):
  """ Runs Mask model and returns detection data for further analysis

      Input: captured image
      Output: Dictionary of detections containing detection bounding 
              box coordinates and detection confidence; have ordered s.t. 
              detections[0] has highest confidence and detections[-1] has 
              lowest confidence 
  """
  # load pre-trained Mask R-CNN model 
  cfg = get_cfg() 
  cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
  #cfg.MODEL.WEIGHTS = os.path.join(cfg.OUTPUT_DIR, model_filename)  # path to the model we just trained
  #cfg.MODEL.WEIGHTS = "/home/xiaobao/InspectionSatCV/ExtractionScripts/model_0004999.pth"
  # FOR DEBUGGING ONLY
  #cfg.MODEL.DEVICE = 'cpu'


  cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1  # only has one class (ballon). (see https://detectron2.readthedocs.io/tutorials/datasets.html#update-the-config-for-new-datasets)
  os.makedirs(cfg.OUTPUT_DIR, exist_ok=True)

  # inference the image 
  cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = inference_threshold   # set a custom testing threshold
  predictor = DefaultPredictor(cfg)

  outputs = predictor(im)       #dict, colab format --> {'instances': Instances(..., fields =[pred_boxes:..., scores:...])}

  # create dicitonary of detections made
  # format --> {0: {'bbox': [3690.1355, 398.60562, 3997.3171, 568.5226], 'conf': 1.0, 1:...}
  detect_bbox = outputs["instances"].pred_boxes.tensor.cpu().numpy()
  detect_conf = outputs["instances"].scores.cpu().numpy()
  num_detect = len(outputs["instances"].pred_boxes.tensor)

  # gather bbox coordinates and confidence for each deteciton per image
  det_dict = {}
  for i in range(num_detect):
    det_dict[i] = {"bbox":{}, "conf":{}}
    det_dict[i]["bbox"] = list(detect_bbox[i])
    det_dict[i]["conf"] = round(detect_conf[i],5)


# ### INPUT VARIABLES FOR TESTING ###
# #model_filename = "/home/xiaobao/InspectionSatCV/ExtractionScripts/model_0004999.pth"    #located in output.zip folder
# im = cv2.imread("capImg_multi.jpg")     #located in same directory as inference_mask.py
# inference_threshold = 0.02
# det_dict = Inference_Mask(im, inference_threshold)


# ### TABULATED DETECTION RESULTS ###
# from tabulate import tabulate

# spc1 = "                 "
# spc2 = "              "

# # Includes IOU and TP?
# headers = ["Detection","Bbox"+spc1+spc1+spc1+"Conf"+ spc2+"IOU"+spc2+"TP?"] 
# table = tabulate(det_dict.items(),headers = headers)
# print(table)