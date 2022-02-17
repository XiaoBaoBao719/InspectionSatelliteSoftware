# ExtractionScripts Instructions

The ExtractionScripts folder consists of the python scripts making up the Science FSW to
be used for the Handrail Inspection in Orbit mission. After image capture, the scripts are 
inteded to be called sequentially: truthRGB.py, inference_Mask.py, inference_YOLO.py, 
performance_Mask.py, performance_YOLO.py, falseposInspect_Mask.py, falseposInspect_YOLO.py,
and outString.py.