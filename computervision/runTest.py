from cv2 import threshold
from Detector import *

# ModelURL = "http://download.tensorflow.org/models/object_detection/tf2/20200711/ssd_mobilenet_v2_fpnlite_640x640_coco17_tpu-8.tar.gz"
ModelURL = "http://download.tensorflow.org/models/object_detection/tf2/20200711/efficientdet_d5_coco17_tpu-32.tar.gz"

classFile = "coco.names"
videoPath = 0
threshold = 0.5

detector = Detector()
detector.readClasses(classFile)
detector.downloadModel(ModelURL)
detector.loadModel()
detector.predictVideo(videoPath, threshold)
