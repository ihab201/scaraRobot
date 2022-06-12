import cv2
import numpy as np

image = cv2.imread('test2.jpg')
#cv2.imshow("Image",image)

#initialize first frame and contours for empty squares
first_frame = None
fixed_contours = None
max_area = 0 
c = 0
max_frame = 0

sharpening_kernel = np.array([[0, -1, 0],
                    [-1, 5,-1],
                    [0, -1, 0]])

kernel = np.ones((10,20),np.uint8)

    
# sharp_img = cv2.filter2D(image, -1, sharpening_kernel)    
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# gray = cv2.morphologyEx(gray,cv2.MORPH_CLOSE,kernel)
blur = cv2.GaussianBlur(gray, (5,5),0)
#cv2.imshow("blur", blur)
blur = cv2.Canny(blur, 10,10)
#cv2.imshow("canny",blur)

thresh = cv2.adaptiveThreshold(blur, 300, 1,1,11, 2)

cv2.imshow("thresh",thresh)

countours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#getting biggest area
for i in countours: 
    area = cv2.contourArea(i)
    if area > max_area:
        if area> max_area : 
            max_area = area
            best_cnt = i
            #org_image = cv2.drawContours(image, countours, c, (0,255,0), 5)
    c+=1

#cv2.imshow("contours",org_image)
mask = np.zeros((gray.shape), np.uint8)
cv2.drawContours(mask, [best_cnt], 0,255,-1)
cv2.drawContours(mask, [best_cnt], 0, 0, 2)
cv2.imshow("mask", mask)

out = np.zeros_like(gray)
out[mask == 255] = gray[mask == 255]
#cv2.imshow("New Image", out)

########## Detecting squares ################
image_cropped = cv2.imread('test3.png')


resharp_img = cv2.filter2D(image_cropped, -1, sharpening_kernel)
blur = cv2.GaussianBlur(image, (5,5), 0)
blur = cv2.GaussianBlur(blur, (5,5), 1)
#blur =cv2.medianBlur(blur, 1)

#cv2.imshow("New blur", blur)
thresh = cv2.adaptiveThreshold(blur, 300, cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY_INV,11, 2)
#cv2.imshow("New Thresh", thresh)


countours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
c=0
if (fixed_contours is None) and (first_frame is None): 
    fixed_contours = countours
    first_frame = out
    
    #in a loop use "continue" at the end.

####object detection within each array... 
absolute_difference = cv2.absdiff(first_frame, out)
_, absolute_difference = cv2.threshold(absolute_difference, 95, 255, cv2.THRESH_BINARY)

#get objects contours.
objContours, objH = cv2.findContours(absolute_difference, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
number_of_objects = len(objContours)
print("number of contours detected: ", str(number_of_objects))

for i in fixed_contours:
    area = cv2.contourArea(i)
    if area > max_area:
        max_area = area
print(max_area)

for i in fixed_contours:
    area = cv2.contourArea(i)
    #getting coordinates of each square
    x,y,w,h = cv2.boundingRect(i)
    if (area > 9000) and (area < 20000):
        cv2.rectangle(image, (x,y), (x+w, y+h), (0,255,0), 15)
        for cnt in objContours:
            objx, objy, objw, objh = cv2.boundingRect(cnt)
            if objx < x and objy < y and (objx + objw) < (x +w) and (objy+objh)<(y+h):
                                cv2.rectangle(image, (x, y),(x+w, y+ h), (255,0,0), 10)
                                
        #cv2.drawContours(image, countours,c, (0,255,0), 2)
        
    c+= 1

cv2.imshow("Squares Det", image)

cv2.waitKey()

cv2.destroyAllWindows()