from cgitb import reset
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

img = cv.imread("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/IMG_1100.jpeg")
print(img.shape)

res = cv.resize(img,None,fx=0.2, fy=0.2, interpolation = cv.INTER_AREA)
print(res.shape)

gray = cv.cvtColor(res, cv.COLOR_BGR2GRAY)

hist = cv.calcHist([gray],[0],None,[256],[0,256])
#plt.plot(hist)
#plt.show()

ret,th1 = cv.threshold(gray,90,255,cv.THRESH_BINARY)

kernel = np.ones((5,5),np.uint8)
opening = cv.morphologyEx(th1, cv.MORPH_OPEN, kernel)

med = cv.medianBlur(opening, 13)
gaus = cv.GaussianBlur(med,(5,5),2)

contours, hierarchy = cv.findContours(gaus, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

temp_cont = () 
for i in range(len(contours)):
    cnt = contours[i]
    if cv.contourArea(cnt) > 1500 and cv.contourArea(cnt) < 100000:
        cont = list(temp_cont)
        cont.append(cnt)
        temp_cont = tuple(cont)

rect_temp = res.copy()

positions = {}
positions["center"] = []
positions["area"] = []  

center_coord = []
center_int = []
differences = []
ratio = []   

for i in range(len(temp_cont)):
    cnt = temp_cont[i]

    x,y,w,h = cv.boundingRect(cnt)
    cv.rectangle(rect_temp,(x,y),(x+w,y+h),(0,0,255),2)

    center_coord.append((round(x+(0.5*w)),round(y+(0.5*h))))
    cv.circle(rect_temp, center_coord[i], 2, (0,0,255), 1)

    center_int.append(gaus.item(center_coord[i]))

    differences.append(abs((w*h)-cv.contourArea(cnt)))

    ratio.append(w/h)

    font = cv.FONT_HERSHEY_SIMPLEX
    cv.putText(rect_temp,str(i),center_coord[i], font, 1,(255,255,255),2,cv.LINE_AA)

print("Area Differences:")
print(differences)
print("Center Intensities:")
print(center_int)
print("w/h Ratios:")
print(ratio)

new_cont = ()
for i in range(len(temp_cont)):
    cnt = temp_cont[i]
    if differences[i] < 1000: #and center_int[i] > 200:
        if ratio[i] < 1.1 and ratio[i] > 0.9:
            cont = list(new_cont)
            cont.append(cnt)
            new_cont = tuple(cont)

rect_post = res.copy()

for i in range(len(new_cont)):
    cnt = new_cont[i]
    x,y,w,h = cv.boundingRect(cnt)
    cv.rectangle(rect_post,(x,y),(x+w,y+h),(0,0,255),2)
    positions["center"].append((round(x+(0.5*w)),round(y+(0.5*h))))
    positions["area"].append(w*h)

print(positions)

conts_pre = res.copy()
cv.drawContours(conts_pre, contours, -1, (0,255,0), 3)
conts_temp = res.copy()
cv.drawContours(conts_temp, temp_cont, -1, (0,255,0), 3)
conts_post = res.copy()
cv.drawContours(conts_post, new_cont, -1, (0,255,0), 3)

show = "following"

if show == "single":
    cv.imshow("Display window", opening)
    k = cv.waitKey(0)

if show == "together":
    titles = ['orginal','Gray Image', 'Global Threshold', 'Median Blur', 'Gaussian Blur', 'Contours Pre', 'Contours Post', 'Rectangles']
    images = [res, gray, th1, med, gaus, conts_pre, conts_post, rect_post]
    for i in range(len(images)):
        plt.subplot(3,3,i+1),plt.imshow(images[i],'gray')
        plt.title(titles[i])
        plt.xticks([]),plt.yticks([])
    plt.show()

if show == "following":
    cv.imshow("Display window", res)
    k = cv.waitKey(0)
    cv.imshow("Display window", gray)
    k = cv.waitKey(0)
    cv.imshow("Display window", th1)
    k = cv.waitKey(0) 
    cv.imshow("Display window", opening)
    k = cv.waitKey(0)
    cv.imshow("Display window", med)
    k = cv.waitKey(0)
    cv.imshow("Display window", gaus)
    cv.imshow("Display window2", conts_pre)
    k = cv.waitKey(0)
    cv.imshow("Display window2", conts_temp)
    cv.imshow("Display window", rect_temp)
    k = cv.waitKey(0)
    cv.imshow("Display window2", conts_post)
    cv.imshow("Display window", rect_post)
    k = cv.waitKey(0)