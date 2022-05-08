
# Some basic setup:
# Setup detectron2 logger
import detectron2
from detectron2.utils.logger import setup_logger
setup_logger()

# import some common libraries
import numpy as np
import os, json, cv2, random
#from google.colab.patches import cv2_imshow

# import some common detectron2 utilities
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog, DatasetCatalog

from detectron2.structures import BoxMode

MODEL_WEIGHTS_PATH = os.getcwd() + "/handrail_model.pth"
print(type(MODEL_WEIGHTS_PATH))

def get_handrail_dicts(img_dir):
    json_file = os.path.join(img_dir, "via_region_data.json")
    with open(json_file) as f:
        imgs_anns = json.load(f)

    dataset_dicts = []
    for idx, v in enumerate(imgs_anns.values()):
        record = {}
        
        filename = os.path.join(img_dir, v["filename"])
        height, width = cv2.imread(filename).shape[:2]
        
        print(height, width)
        
        record["file_name"] = filename
        record["image_id"] = idx
        record["height"] = height
        record["width"] = width
      
        annos = v["regions"]
        objs = []
        for _, anno in annos.items():
            #assert not anno["region_attributes"]
            anno_shp = anno["shape_attributes"]
            px = anno_shp["all_points_x"]
            py = anno_shp["all_points_y"]
            poly = [(x + 0.5, y + 0.5) for x, y in zip(px, py)]
            poly = [p for x in poly for p in x]
            h_class = anno["region_attributes"]["title"]

            cat_num = 0
            if h_class == "handrail_cone":
                cat_num = 0
            if h_class == "handrail_ring":
                cat_num = 1
            if h_class == "handrail_box":
                cat_num = 2

            obj = {
                "bbox": [np.min(px), np.min(py), np.max(px), np.max(py)],
                "bbox_mode": BoxMode.XYXY_ABS,
                "segmentation": [poly],
                "category_id": cat_num,
            }

            objs.append(obj)
        record["annotations"] = objs
        dataset_dicts.append(record)
    return dataset_dicts

def detect(filename):
    test_data = [{'file_name': filename}]

    #inputPath = '/home/pi/HandrailData/observations/handrail-input.jpg'
    #samplePath = '/home/pi/Mask_Handrail/input1.jpg'

    #test_data = [{'file_name': samplePath
    #            }]

    im = cv2.imread(test_data[0]["file_name"])
    outputs = predictor(im)
    v = Visualizer(im[:, :, ::-1],
                    metadata=handrail_metadata, 
                    scale=0.5, 
                        # remove the colors of unsegmented pixels. This option is only available for segmentation models
        )
    out = v.draw_instance_predictions(outputs["instances"].to("cpu"))

    img_out = out.get_image()[:, :, ::-1]
    imS = cv2.resize(img_out, (800, 600))
    cv2.imshow('',imS)

    cv2.imwrite('/home/pi/handrail-output.jpg', imS)
    cv2.waitKey(0) == 27

    print("Returns DATAFRAME")


# Iterate through dictionary of ground truths and labels
for d in ["train", "val"]:
    DatasetCatalog.register("handrail_" + d, lambda d=d: get_handrail_dicts("handrail5/" + d))  #.# folder specific
    MetadataCatalog.get("handrail_" + d).set(thing_classes=["Cone","Ring","Box"])
handrail_metadata = MetadataCatalog.get("handrail_train")

# Setup inference configuration parameters
cfg = get_cfg()
cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
cfg.MODEL.WEIGHTS = MODEL_WEIGHTS_PATH # Set path model .pth
cfg.MODEL.DEVICE = 'cpu'
#cfg.DATASETS.TRAIN = ("handrail_train",)
#cfg.DATASETS.TEST = ("handrail_val",)
#cfg.DATALOADER.NUM_WORKERS = 2

cfg.SOLVER.IMS_PER_BATCH = 5

#cfg.SOLVER.BASE_LR = 0.0125  # pick a good LR
#cfg.SOLVER.MAX_ITER = 30000    # 300 iterations seems good enough for this toy dataset; you will need to train longer for a practical dataset
#cfg.SOLVER.STEPS = []        # do not decay learning rate
##cfg.MODEL.ROI_HEADS.BATCH_SIZE_PER_IMAGE = 128   # faster, and good enough for this toy dataset (default: 512)
cfg.MODEL.ROI_HEADS.NUM_CLASSES = 3  # only has one class (ballon). (see https://detectron2.readthedocs.io/tutorials/datasets.html#update-the-config-for-new-datasets)
# NOTE: this config means the number of classes, but a few popular unofficial tutorials incorrect uses num_classes+1 here.
cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5 # Set threshold for this model
predictor = DefaultPredictor(cfg)


if __name__ == "__main__":
    pass