""" Pass Image into Mask R-CNN Model --> code within this script will become funciton "Inference_Mask" that returns type dictionary "det_dict"

Runs Mask model and returns detection data for further analysis

Input [type XX]: captured image
Output [type dict]: Dictionary of detections containing detection bounding 
        box coordinates and detection confidence; have ordered s.t. 
        detections[0] has highest confidence and detections[-1] has 
        lowest confidence 

Author(s): Billy Orion Mazotti

Last Edited: 2/6/22

WARNING: currently using stand-ins for captured image and CNN model.pth
"""


# imports
""" [NEEDS DEPENDENCIES SATISFIED --> @XB]
Functions that need library: get_cfg(), DefaultPredictor(), cv2.imread()

from detectron2.config import get_cfg
from detectron2.engine import DefaultPredictor
import cv2
"""


model_filename = "model_0004999.pth"    #located in output.zip folder
im = cv2.imread("capImg_multi.jpg")     #located in same directory as inference_mask.py
inference_threshold = 0.02


# load pre-trained Mask R-CNN model 
cfg = get_cfg()
cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
cfg.MODEL.WEIGHTS = os.path.join(cfg.OUTPUT_DIR, model_filename)  # path to the model we just trained
cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1  # only has one class (ballon). (see https://detectron2.readthedocs.io/tutorials/datasets.html#update-the-config-for-new-datasets)
os.makedirs(cfg.OUTPUT_DIR, exist_ok=True)

# inference the image 
cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = inference_threshold   # set a custom testing threshold
predictor = DefaultPredictor(cfg)

outputs = predictor(im)       #dict, colab format --> {'instances': Instances(..., fields =[pred_boxes:..., scores:...])}

# create dicitonary of detections made (based on colab work, may need changing if )
# format --> {0: {'bbox': [3690.1355, 398.60562, 3997.3171, 568.5226], 'conf': 1.0, 1:...}
detect_bbox = outputs["instances"].pred_boxes.tensor.cpu().numpy()
detect_conf = outputs["instances"].scores.cpu().numpy()
num_detect = len(outputs["instances"].pred_boxes.tensor)

# gather bbox coordinates and confidence for each deteciton per image ((based on colab work))
det_dict = {}
for i in range(num_detect):
  det_dict[i] = {"bbox":{}, "conf":{}}
  det_dict[i]["bbox"] = list(detect_bbox[i])
  det_dict[i]["conf"] = round(detect_conf[i],5)

