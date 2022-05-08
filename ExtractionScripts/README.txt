#.# ExtractionScripts Instructions

The ExtractionScripts folder consists of the python scripts making up the Science FSW to
be used for the Handrail Inspection in Orbit mission. After image capture, the scripts are 
inteded to be called sequentially: truthRGB.py, inference_Mask.py, inference_YOLO.py, 
performance.py, falseposInspect.py, Pixel_Extraction.py and outString.py.

# FOR FUTURE REQUIREMENTS DOC (AUTOMATE DEPENDENCY INSTALLATION)
#.# VIRTUAL ENVIRONMENT PACKAGES/DEPENDENCIES
SCRIPT --> IMPORT       PACKAGE NAME            VERSION                 BUILD       CHANNEL
all scripts
--> python              python                    3.7.11               h6244533_0           

truthRGB.py
--> cv2                 opencv-python             4.5.3.56                 pypi_0    pypi
--> numpy               numpy                     1.21.5           py37h5fa1a60_0    conda-forge

inference_Mask.py
--> detectron2:     ???
--> cv2

inference_YOLO.py
--> 

performance.py
--> 

falseposInspect.py
--> 


Pixel_Extraction.py
--> 

outString.py
--> 