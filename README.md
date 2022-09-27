# InspectionSatelliteSoftware
This is where the HRVIP Lab maintains the code base for the InspectionSat Hard Disk Drive (HDD) and Handrail Identification in Orbit (HIO) experiments that are slated to fly with the R5S4 6U spacecraft from the Seeker Lab at the NASA Johnson Space Center. The HDD experiment is a technology demonstrator that will advance the Technology Readiness Level (TRL) for the utilization of low-cost, commcerically off the shelf (COTS), hard drives as reaction control wheels for small satellites. The HIO experiment is also a technology demonstration mission that aims to provide a proof-of-concept application of deep learning deployed on (COTS) edge compute x64 ARM-based devices in space. It will utilize two unique deep learning frameworks in order to identify and localize scale model handrails as feature landmarks for pose estimation of spacecraft orbiting the International Space Station in future work.

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
