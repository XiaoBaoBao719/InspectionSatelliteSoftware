""" Pass Image into Mask R-CNN Model

Runs Mask model and returns detection data for further analysis

Input [type XX]: captured image
Output [type dict]: Dictionary of detections containing detection bounding 
        box coordinates and detection confidence; have ordered s.t. 
        detections[0] has highest confidence and detections[-1] has 
        lowest confidence 

Author(s): Billy Orion Mazotti

Last Edited: 1/29/22
"""

# imports
""" [NEEDS DEPENDENCIES SATISFIED --> @XB]
"""
import cv2

 
# load pre-trained Mask R-CNN model (based on colab work)
""" [NEEDS .pth MODEL FILE --> @YC]
cfg.MODEL.WEIGHTS = os.path.join(cfg.OUTPUT_DIR, "[MODEL_FILE.pth]")  # path to the model we just trained
"""

# inference the image (based on colab work)
""" [UNCOMMENT AFTER ADDING DEPENDENCIES AND MODEL.pth]
cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5   # set a custom testing threshold
predictor = DefaultPredictor(cfg)
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

im = cv2.imread("dmyImgCap.jpg")
print(type(im))