import cv2
import numpy as np
import time 




cap = cv2.VideoCapture(0)
# image = cv2.imread('test2.jpg')

#initialize first frame and contours for empty squares
first_frame = None
fixed_contours = None

#inhancements
# fixed_mask = None

max_area = 0 
c = 0


sharpening_kernel = np.array([[0, -1, 0],
                    [-1, 5,-1],
                    [0, -1, 0]])

kernel_dilate = np.ones((3))
kernel = np.ones((10,4),np.uint8)

while True: 
    
    _, image = cap.read()

    #cv2.imshow("Image",image)

    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #gray = cv2.morphologyEx(gray,cv2.MORPH_CLOSE,kernel)
    blur = cv2.GaussianBlur(gray, (5,5),1)
    #cv2.imshow("blur", blur)

    thresh = cv2.adaptiveThreshold(blur, 255, 1,1,11, 2)
    thresh = cv2.dilate(thresh, kernel_dilate, iterations=1)

    #cv2.imshow("thresh",thresh)

    countours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #getting biggest area
    for i in countours: 
        area = cv2.contourArea(i)
        if area> max_area and (area < 2000000): 
             max_area = area
             best_cnt = i
             #image = cv2.drawContours(image, countours, c, (0,255,0), 5)
        c+=1

    #cv2.imshow("contours",image)
    mask = np.zeros((gray.shape), np.uint8)
    cv2.drawContours(mask, [best_cnt], 0,255,-1)
    cv2.drawContours(mask, [best_cnt], 0, 0, 2)
    cv2.imshow("mask", mask)

    # if fixed_mask is None:
    #     fixed_mask = mask
    
    out = np.zeros_like(gray)
    out[mask == 255] = gray[mask == 255]
    cv2.imshow("New Image", out)

    # out = np.zeros_like(gray)
    # out[fixed_mask == 255] = gray[fixed_mask == 255]
    # cv2.imshow("New Image", out)

    ########## Detecting squares ################
    if first_frame is None:
        resharp_img = cv2.filter2D(out, -1, sharpening_kernel)
    else:
        resharp_img = cv2.filter2D(first_frame, -1, sharpening_kernel)
    
    blur = cv2.GaussianBlur(resharp_img, (5,5), 1)
    blur = cv2.GaussianBlur(blur, (5,5), 4)

    #cv2.imshow("New blur", blur)
    thresh = cv2.adaptiveThreshold(blur, 300, 1,1,11, 2)
    #thresh = cv2.Canny(blur, 150,80)
    thresh = cv2.dilate(thresh, np.ones((3)), iterations=1)

    cv2.imshow("New Thresh", thresh)


    countours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    c=0
    if (fixed_contours is None) and (first_frame is None): 
        fixed_contours = countours
        first_frame = out
        continue
        #in a loop use "continue" at the end.

    ####object detection within each array... 
    absolute_difference = cv2.absdiff(first_frame, out)
    _, absolute_difference = cv2.threshold(absolute_difference, 95, 255, cv2.THRESH_BINARY)
    cv2.imshow("absdiff", absolute_difference)

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
        
        if (area > 6000) and (area < 48000):
            cv2.rectangle(image, (x,y), (x+w, y+h), (0,255,0), 8)
            #cv2.drawContours(image, countours,c, (0,255,0), 2)
            for cnt in objContours:
                objx, objy, objw, objh = cv2.boundingRect(cnt)
                if (objx > x) and (objy > y) and (objy < (y + h)) and (objx < (x + w)) :
                        cv2.rectangle(image, (x, y),(x+w, y+ h), (0,0,255), 12)
                                    
            #cv2.drawContours(image, countours,c, (0,255,0), 2)
            
        c+= 1

    cv2.imshow("Squares Det", image)

    key = cv2.waitKey(1)
    if key == ord("r"):
        first_frame = None 
        fixed_contours = None 
        # fixed_mask = None
        continue
    if key == ord("q"):
        break

cv2.destroyAllWindows()

# and (objx + objw) < (x +w) and (objy+objh)<(y+h)
