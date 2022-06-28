import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

img = cv.imread("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/IMG_0996.jpg")
print(img.shape)

res = cv.resize(img,None,fx=0.2, fy=0.2, interpolation = cv.INTER_AREA)
print(res.shape)

gray = cv.cvtColor(res, cv.COLOR_BGR2GRAY)

ret,th1 = cv.threshold(gray,75,255,cv.THRESH_BINARY)
th2 = cv.adaptiveThreshold(gray,255,cv.ADAPTIVE_THRESH_MEAN_C,\
            cv.THRESH_BINARY,11,4)
th3 = cv.adaptiveThreshold(gray,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv.THRESH_BINARY,11,2)


med = cv.medianBlur(th1,13)
gaus = cv.GaussianBlur(med,(5,5),2)
lat = cv.bilateralFilter(gaus,9,75,75)



kernel = np.ones((3,3),np.uint8)
closing = cv.morphologyEx(th3, cv.MORPH_CLOSE, kernel)

laplacian = cv.Laplacian(gray,cv.CV_64F)

edges = cv.Canny(gray,100,200)

contours, hierarchy = cv.findContours(gaus, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
#cv.drawContours(res, contours, -1, (0,255,0), 3)


#titles = ['orginal','Gray Image', 'Global Threshold', 'Adaptive Mean Thresholding', 'Adaptive Gaussian Thresholding', 'Gaussian Blur', 'Median Blur', 'BiLat Filter', 'Closing', 'Edges']
#images = [res, gray, th1, th2, th3, gaus, med, lat, closing, edges]
#for i in range(len(images)):
#    plt.subplot(3,4,i+1),plt.imshow(images[i],'gray')
#    plt.title(titles[i])
#    plt.xticks([]),plt.yticks([])
#plt.show()

new_cont = ()
for i in range(len(contours)):
    cnt = contours[i]
    if cv.contourArea(cnt) > 750 and cv.contourArea(cnt) < 350000:
        cont = list(new_cont)
        cont.append(cnt)
        new_cont = tuple(cont)

for i in range(len(new_cont)):
    cnt = new_cont[i]
    x,y,w,h = cv.boundingRect(cnt)
    cv.rectangle(res,(x,y),(x+w,y+h),(0,0,255),2)
    print(cv.contourArea(cnt))

cv.drawContours(res, new_cont, -1, (0,255,0), 3)

print(len(contours))
print(len(new_cont))

cv.imshow("Display window", res)
k = cv.waitKey(0)