# InspectionSatCV
Maintains the code base for the InspectionSat HDD and HIO experiments that are slated to fly with the R5S4 6U spacecraft from the Seeker Lab at the NASA Johnson Space Center.

## Table of Contents
- Installation
- How to use this repo
- Future Work
- Sources


## Installation


## How to use this repo


## Future Work


## Sources

## Detectron2 
To compute mAP after a certain number of iteration, we need to use MyTrainer function instead of DefaultTrainer. Follow the steps:
1. Train with DefaultTrainer. Comment out the evaluation period, and set the max iteration to a small number. 20 should work.
2. After training, model_final.pth should be in the output folder. Run the code under Inference & evaluation using the trained model section. This should save two handrail coco data files in the output folder.
3. Delete all other files except the two handrail coco data. These two files are used to evaluate mAP in MyTrainer
4. Navigate back to Train section. Set the evaluation period. And use MyTrainer function. Training should start and evaluation will be performed according to the evaluation period you specified.

## YOLOv5
