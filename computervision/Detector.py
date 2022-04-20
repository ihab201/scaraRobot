from email.mime import image
import cv2, time, os, tensorflow as tf
import numpy as np
from tensorflow.python.keras.utils.data_utils import get_file

np.random.seed(20)

class Detector: 
    def __init__(self):
        pass
    def readClasses(self, classesFilepath):
        with open(classesFilepath, 'r') as f: 
            self.classesList = f.read().splitlines() 

        #Color list 
        self.colorList = np.random.uniform(low=0, high= 255, size=(len(self.classesList), 3))

    def downloadModel(self, modelURL):
        #extracting file name.
        fileName = os.path.basename(modelURL)
        self.modelName = fileName[:fileName.index('.')]

        #make a directory to store pretrained models
        self.cacheDir = "./pretrained_models"
        os.makedirs(self.cacheDir, exist_ok=True)

        get_file(fname = fileName, origin = modelURL, cache_dir = self.cacheDir,
         cache_subdir ="checkpoints", extract = True)
    
    def loadModel(self):
        print("Loading Model "+ self.modelName)
        tf.keras.backend.clear_session()
        self.model = tf.saved_model.load(os.path.join(self.cacheDir, "checkpoints", self.modelName,"saved_model"))
        print("model "+ self.modelName+ " is loaded successfully")
    
    def createBoundingBox(self, image, threshold = 0.5):
        inputTensor = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2RGB)
        inputTensor = tf.convert_to_tensor(inputTensor, dtype = tf.uint8)
        inputTensor = inputTensor[tf.newaxis, ...]

        detetctions = self.model(inputTensor)

        bboxs = detetctions['detection_boxes'][0].numpy()
        #extracting class predictions..
        classIndexes = detetctions['detection_classes'][0].numpy().astype(np.int32)
        #extracting the confidence scores for each class label
        classScores = detetctions['detection_scores'][0].numpy()

        imH, imW, imC = image.shape #imC :number of channels

        bboxIdx = tf.image.non_max_suppression(bboxs, classScores, max_output_size=50,
        iou_threshold=threshold, score_threshold=threshold)

        if len(bboxIdx) != 0:
            for i in bboxIdx:
                bbox = tuple(bboxs[i].tolist())
                classConfidence = round(100*classScores[i])
                classIndex = classIndexes[i]

                classLabeText = self.classesList[classIndex].upper()
                classColor = self.colorList[classIndex]

                displayText = '{}: {}%'.format(classLabeText, classConfidence)

                ymin, xmin, ymax, xmax = bbox

                xmin, xmax, ymin, ymax = (xmin* imW, xmax * imW, ymin* imH, ymax* imH)
                xmin, xmax, ymin, ymax = int(xmin), int(xmax), int(ymin), int(ymax)

                cv2.rectangle(image, (xmin,ymin), (xmax, ymax), color = classColor, thickness= 1)
                cv2.putText(image, displayText, (xmin, ymin - 10), cv2.FONT_HERSHEY_PLAIN,1, classColor, 2)

                lineWidth = min(int((xmax - xmin) * 0.2) , int((ymax-ymin)* 0.2))
                #1st corner
                cv2.line(image, (xmin, ymin), (xmin+ lineWidth, ymin), classColor, thickness= 5)
                cv2.line(image, (xmin, ymin), (xmin, ymin+ lineWidth), classColor, thickness= 5)
                #2nd corner
                cv2.line(image, (xmax, ymin), (xmax- lineWidth, ymin), classColor, thickness= 5)
                cv2.line(image, (xmax, ymin), (xmax, ymin+ lineWidth), classColor, thickness= 5)
                #3rd corner
                cv2.line(image, (xmin, ymax), (xmin+ lineWidth, ymax), classColor, thickness= 5)
                cv2.line(image, (xmin, ymax), (xmin, ymax- lineWidth), classColor, thickness= 5)
                #4th corner
                cv2.line(image, (xmax, ymax), (xmax- lineWidth, ymax), classColor, thickness= 5)
                cv2.line(image, (xmax, ymax), (xmax, ymax- lineWidth), classColor, thickness= 5)
                
        
        return image

    def predictVideo(self, videoPath, threshold= 0.5):
        cap = cv2.VideoCapture(videoPath)

        if (cap.isOpened() == False):
            print("error opening video..")
            return
        
        (success, image) = cap.read()
        startTime = 0 

        while success: 
            currentTime = time.time()
            fps = 1/(currentTime - startTime)
            startTime = currentTime

            bboxImage = self.createBoundingBox(image, threshold)

            cv2.putText(bboxImage, "FPS: "+ str(int(fps)), (20, 70), cv2.FONT_HERSHEY_PLAIN, 2,(0,255,0),2)
            cv2.imshow("Result", bboxImage)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break
            (success, image) = cap.read()
        cv2.destroyAllWindows()