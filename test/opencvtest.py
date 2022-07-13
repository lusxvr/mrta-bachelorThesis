import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

img = cv.imread("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/IMG_1100.jpeg")
print(img.shape)

res = cv.resize(img,None,fx=0.2, fy=0.2, interpolation = cv.INTER_AREA)
print(res.shape)

gray = cv.cvtColor(res, cv.COLOR_BGR2GRAY)

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
differences = []
ratio = []
histogramms = []
max_intensity = []

dict_rec = {}
dict_rec["x"] = []
dict_rec["y"] = []
dict_rec["w"] = []
dict_rec["h"] = [] 

for i in range(len(temp_cont)):
    cnt = temp_cont[i]

    #Calculating and drawing the Bounding Rectangel
    x,y,w,h = cv.boundingRect(cnt)
    cv.rectangle(rect_temp,(x,y),(x+w,y+h),(0,0,255),2)

    #Calculating and drawing the Center Point of the Rectangle
    center_coord.append((round(x+(0.5*w)),round(y+(0.5*h))))
    cv.circle(rect_temp, center_coord[i], 2, (0,0,255), 1)

    #Calculating the Area Difference between Contour and Bounding Rectangle
    differences.append(abs((w*h)-cv.contourArea(cnt)))

    #Calculating the w/h Ratio of the Bounding Rectangle
    ratio.append(w/h)

    #Creating the Histogramm for each Bounding Rectangle, normalized withe the area
    mask = np.zeros(res.shape[:2], np.uint8)
    mask[y:(y+h), x:(x+w)] = 255
    masked_img = cv.bitwise_and(res,res,mask = mask)
    hist_mask = cv.calcHist([gray],[0],mask,[255],[0,256]) / (w*h)
    histogramms.append(hist_mask)
    
    #Calculating which Intensity is the most common in the histogramms
    max_intensity.append(np.average(np.where(hist_mask == np.amax(hist_mask))[0]))

    dict_rec["x"].append(x)
    dict_rec["y"].append(y)
    dict_rec["w"].append(w)
    dict_rec["h"].append(h)

    #Numbering the Bounding Rectangles
    font = cv.FONT_HERSHEY_SIMPLEX
    cv.putText(rect_temp,str(i),center_coord[i], font, 1,(255,255,255),2,cv.LINE_AA)

print("Area Differences:")
print(differences)
print("w/h Ratios:")
print(ratio)
print("Peaks of Histogramms for Bounding Rectangle of each Contour:")
print(max_intensity)

"""
for i in range(len(dict_rec["x"])):
    mask = np.zeros(res.shape[:2], np.uint8)
    mask[dict_rec["y"][i]:dict_rec["y"][i]+dict_rec["h"][i], dict_rec["x"][i]:dict_rec["x"][i]+dict_rec["w"][i]] = 255
    masked_img = cv.bitwise_and(res,res,mask = mask)
    hist_mask = cv.calcHist([gray],[0],mask,[255],[0,256]) / (dict_rec["w"][i]*dict_rec["h"][i])
    histogramms.append(hist_mask)
    
    max_intensity.append(np.average(np.where(hist_mask == np.amax(hist_mask))[0]))
"""

#Plotting the Histogramms
#plt.subplots()
#for i in range(len(histogramms)):
#    plt.plot(histogramms[i], label=i)
#plt.legend()
#plt.savefig("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/Results/histogram.png")
#plt.show()

new_cont = ()
for i in range(len(temp_cont)):
    cnt = temp_cont[i]
    if differences[i] < 1000 and max_intensity[i] > 100:
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

print("Final Calculated free spaces:")
print(positions)

conts_pre = res.copy()
cv.drawContours(conts_pre, contours, -1, (0,255,0), 3)
conts_temp = res.copy()
cv.drawContours(conts_temp, temp_cont, -1, (0,255,0), 3)
conts_post = res.copy()
cv.drawContours(conts_post, new_cont, -1, (0,255,0), 3)

show = ""

if show == "single":
    cv.imshow("Display window", masked_img)
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

if show == 'save':
    cv.imwrite("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/Results/gray.jpeg", gray)
    cv.imwrite("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/Results/threshold.jpeg", th1)
    cv.imwrite("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/Results/opening.jpeg", opening)
    cv.imwrite("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/Results/median.jpeg", med)
    cv.imwrite("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/Results/gaus.jpeg", gaus)
    cv.imwrite("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/Results/contours_pre.jpeg", conts_pre)
    cv.imwrite("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/Results/contours_temp.jpeg", conts_temp)
    cv.imwrite("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/Results/rectangles_temp.jpeg", rect_temp)
    cv.imwrite("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/Results/contours_post.jpeg", conts_post)
    cv.imwrite("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/Results/rectangles_post.jpeg", rect_post)