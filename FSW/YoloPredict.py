""" Pass Image into Yolo v5 Model

Runs model and returns detection data for further analysis

Author(s): Cordell Cohoon

Last Edited: 6/13/22, Xiao-Bao Bao

"""

# Setup 
import yolov5
import os

# load custom trained model weights
model_path = os.getcwd() + '/FSW/yolo_model.pt'
model = yolov5.load(model_path)

# set model parameters
model.conf = 0.4  # NMS confidence threshold
#model.iou = 0.45  # NMS IoU threshold
model.agnostic = False  # NMS class-agnostic
model.multi_label = False  # NMS multiple labels per box
model.max_det = 10  # maximum number of detections per image

def Inference_Yolo(_img):
    # set image
    img = _img

    # perform inference
    results = model(img)

    # inference with larger input size
    #results = model(img, size=1280)

    # parse results into tensor
    predictions = results.pred[0]
    boxes = predictions[:, :4] # x1, y1, x2, y2
    scores = predictions[:, 4]
    categories = predictions[:, 5]

    # show detection bounding boxes on image
    results.show()

    # Pull the top bbox, conf score
    best_box = boxes[0].tolist()
    best_conf = scores[0].item()

    # save results into "results/" folder
    #results.save(save_dir='results/')
    # Convert from tensor
    return best_box, best_conf


## FOR DEBUGGING ONLY ##
# test_image = os.getcwd() + '/FSW/sample_img.jpg'
# result = Inference_Yolo(test_image)
# boxes, conf = result

# print("Boxes: ", boxes, type(boxes))
# print("conf ", conf, type(conf))