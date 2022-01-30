""" Pass Image into Mask R-CNN Model

Runs Mask model and returns detection data for further analysis

Input [type XX]: captured image
Output [type dict]: Dictionary of detections containing detection bounding 
        box coordinates and detection confidence; have ordered s.t. 
        detections[0] has highest confidence and detections[-1] has 
        lowest confidence 

Author(s): Billy Orion Mazotti

Last Edited: 1/29/22

WARNING: currently using stand-ins for captured image and CNN model.pth
"""

# imports
""" [NEEDS DEPENDENCIES SATISFIED --> @XB]
import cv2
"""


""" [UNCOMMENT AFTER ADDING DEPENDENCIES]
# load pre-trained Mask R-CNN model (based on colab work)
cfg.MODEL.WEIGHTS = os.path.join(cfg.OUTPUT_DIR, "dmyModle.pth")  # path to the model we just trained

# inference the image (based on colab work)

cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5   # set a custom testing threshold
predictor = DefaultPredictor(cfg)
im = cv2.imread("dmyImgCap.jpg")
outputs = predictor(im)


# create dicitonary of detections made (based on colab work)
# format --> {0: {'bbox': [3690.1355, 398.60562, 3997.3171, 568.5226], 'conf': 1.0, 1:...}
detect_bbox = outputs["instances"].pred_boxes.tensor.cpu().numpy()
detect_conf = outputs["instances"].scores.cpu().numpy()
num_detect = len(outputs["instances"].pred_boxes.tensor)

# gather bbox coordinates and confidence for each deteciton per image ((based on colab work))
det_dict = {}
for i in range(num_detect):
  det_dict[i] = {"bbox":{}, "conf":{}}
  det_dict[i]["bbox"] = list(detect_bbox[i])
  det_dict[i]["conf"] = round(detect_conf[i],2)
"""

