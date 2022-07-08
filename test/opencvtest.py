from cgitb import reset
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

img = cv.imread("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/IMG_1100.jpeg")
print(img.shape)

res = cv.resize(img,None,fx=0.2, fy=0.2, interpolation = cv.INTER_AREA)
print(res.shape)

gray = cv.cvtColor(res, cv.COLOR_BGR2GRAY) 

ret,th1 = cv.threshold(gray,100,255,cv.THRESH_BINARY)

#th2 = cv.adaptiveThreshold(gray,255,cv.ADAPTIVE_THRESH_MEAN_C,cv.THRESH_BINARY,11,4)
#th3 = cv.adaptiveThreshold(gray,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY,11,2)

med = cv.medianBlur(th1, 13)
gaus = cv.GaussianBlur(med,(5,5),2)

#lat = cv.bilateralFilter(gaus,9,75,75)
#kernel = np.ones((3,3),np.uint8)
#closing = cv.morphologyEx(th3, cv.MORPH_CLOSE, kernel)
#laplacian = cv.Laplacian(gray,cv.CV_64F)
#edges = cv.Canny(gray,100,200)

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
##positions["center_org"] = []
positions["area"] = []  
differences = []
center_int = []
##center_test = np.array([[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]])
center_res = []

##gaus_test = cv.cvtColor(gaus, cv.COLOR_GRAY2BGR)

## print("Center Pixel Values for res and img:")

for i in range(len(temp_cont)):
    cnt = temp_cont[i]
    x,y,w,h = cv.boundingRect(cnt)
    cv.rectangle(rect_temp,(x,y),(x+w,y+h),(0,0,255),2)
    positions["center"].append((round(x+(0.5*w)),round(y+(0.5*h))))
    center_int.append(gaus.item(positions["center"][i]))

    ##center_test[0][i] = gray.item(positions["center"][i])
    ##center_test[1][i] = th1.item(positions["center"][i])
    ##center_test[2][i] = med.item(positions["center"][i])
    ##center_test[3][i] = gaus.item(positions["center"][i])

    ##positions["center_org"].append(tuple(elem_1 * elem_2 for elem_1, elem_2 in zip(positions["center"][i], (5,5))))
    ##print(i)
    ##print(res[positions["center"][i]])
    ##print(img[positions["center_org"][i]])

    cv.circle(rect_temp, positions["center"][i], 2, (0,0,255), 1)
    ##cv.circle(gray, positions["center"][i], 2, (255,255,255), 1)
    ##cv.circle(th1, positions["center"][i], 2, (255,255,255), 1)
    diff = abs((w*h)-cv.contourArea(cnt))
    differences.append(diff)

    font = cv.FONT_HERSHEY_SIMPLEX
    cv.putText(rect_temp,str(i),positions["center"][i], font, 1,(255,255,255),2,cv.LINE_AA)

    
    ##cv.circle(gaus_test, positions["center"][i], 2, (0,0,255), 1)

"""
for i in range(len(center_int)):
    pos = positions["center"][i]
    center_int[i] += 0.2*gaus.item(pos[0]+1,pos[1])
    center_int[i] += 0.2*gaus.item(pos[0]-1,pos[1])
    center_int[i] += 0.2*gaus.item(pos[0],pos[1]+1)
    center_int[i] += 0.2*gaus.item(pos[0],pos[1]-1)
"""
print("Area Differences:")
print(differences)
print("Center Intensities:")
print(center_int)

new_cont = ()
for i in range(len(temp_cont)):
    cnt = temp_cont[i]
    if differences[i] < 1000: #and center_int[i] > 200:
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
    
##print("Intensities for the center pixels in gray, th1, med and gaus:")
##print(center_test)

print(positions)


conts_pre = res.copy()
cv.drawContours(conts_pre, contours, -1, (0,255,0), 3)
conts_temp = res.copy()
cv.drawContours(conts_temp, temp_cont, -1, (0,255,0), 3)
conts_post = res.copy()
cv.drawContours(conts_post, new_cont, -1, (0,255,0), 3)

show = "following"

if show == "single":
    cv.imshow("Display window", th1)
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