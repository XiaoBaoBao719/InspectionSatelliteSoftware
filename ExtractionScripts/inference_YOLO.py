""" Pass Image into YOLO Model

Runs YOLO model and returns detection data for further analysis

Input: captured image
Output: Dictionary of detections containing detection bounding 
        box coordinates and detection confidence ; have ordered s.t. 
        detections[0] has highest confidence and detections[-1] has 
        lowest confidence; write output to detections_Mask.json, 
        detections_YOLO.json
Author(s): 

Last Edited: 1/29/22

"""