import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

#Reading Image and Preprocessing
img_res = cv.imread("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/gray_image_943222070069.png")

#img_res = cv.resize(img,None,fx=0.2, fy=0.2, interpolation = cv.INTER_AREA)

img_gray = cv.cvtColor(img_res, cv.COLOR_BGR2GRAY)

ret,img_th1 = cv.threshold(img_gray,110,255,cv.THRESH_BINARY)

kernel_opening = np.ones((5,5),np.uint8) 
img_open = cv.morphologyEx(img_th1, cv.MORPH_OPEN, kernel_opening)

img_med = cv.medianBlur(img_open, 13)
img_gaus = cv.GaussianBlur(img_med,(5,5),2)

#Calculating Contours from Preprocessed Image
contours_org, hierarchy = cv.findContours(img_gaus, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

#Discarding very big and very small contours
contours_pre = () 
for i in range(len(contours_org)):
    cnt = contours_org[i]
    if cv.contourArea(cnt) > 10000 and cv.contourArea(cnt) < 100000:
        cont = list(contours_pre) 
        cont.append(cnt)
        contours_pre = tuple(cont)

#Calculating Metrics to classify remaining candidates
img_rect_pre = img_res.copy() 

center_coord = []
differences = []
ratios = []
histogramms = []
max_intensity = []

for i in range(len(contours_pre)):
    cnt = contours_pre[i]

    #Calculating and drawing the Bounding Rectangel
    x,y,w,h = cv.boundingRect(cnt)
    cv.rectangle(img_rect_pre,(x,y),(x+w,y+h),(0,0,255),2)

    #Calculating and drawing the Center Point of the Rectangle
    center_coord.append((round(x+(0.5*w)),round(y+(0.5*h))))
    cv.circle(img_rect_pre, center_coord[i], 2, (0,0,255), 1)

    #Calculating the Area Difference between Contour and Bounding Rectangle
    differences.append(abs((w*h)-cv.contourArea(cnt)))

    #Calculating the w/h Ratio of the Bounding Rectangle
    ratios.append(round((w/h), 4))

    #Creating the Histogramm for each Bounding Rectangle, normalized withe the area
    mask = np.zeros(img_res.shape[:2], np.uint8)
    mask[y:(y+h), x:(x+w)] = 255
    masked_img = cv.bitwise_and(img_res,img_res,mask = mask)
    hist_mask = cv.calcHist([img_gray],[0],mask,[255],[0,256]) / (w*h)
    histogramms.append(hist_mask)
    
    #Calculating which Intensity is the most common in the histogramms
    max_intensity.append(np.average(np.where(hist_mask == np.amax(hist_mask))[0]))

    #Numbering the Bounding Rectangles
    font = cv.FONT_HERSHEY_SIMPLEX
    cv.putText(img_rect_pre,str(i),center_coord[i], font, 1,(255,255,255),2,cv.LINE_AA)

print("Area Differences:")
print(differences)
print("w/h Ratios:")
print(ratios)
print("Peaks of Histogramms for Bounding Rectangle of each Contour:")
print(max_intensity)

#Plotting the Histogramms
#plt.subplots()
#for i in range(len(histogramms)):
#    plt.plot(histogramms[i], label=i)
#plt.legend()
#plt.savefig("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/Results/histogram.png")
#plt.show()

#FIltering the Contours according to the calculated metrices
contours_post = ()
for i in range(len(contours_pre)):
    cnt = contours_pre[i]
    if differences[i] < 2000 and max_intensity[i] > 100:
        if ratios[i] < 1.1 and ratios[i] > 0.9:
            cont = list(contours_post)
            cont.append(cnt)
            contours_post = tuple(cont)

#Saving the final Positions
img_rect_post = img_res.copy()

positions = {}
positions["center"] = []
positions["area"] = []  

for i in range(len(contours_post)):
    cnt = contours_post[i]
    x,y,w,h = cv.boundingRect(cnt)
    cv.rectangle(img_rect_post,(x,y),(x+w,y+h),(0,0,255),2)
    positions["center"].append((round(x+(0.5*w)),round(y+(0.5*h))))
    positions["area"].append(w*h)

print("Final Calculated free spaces:")
print(positions)

#Drawing the different Contours
img_contours_org = img_res.copy()
cv.drawContours(img_contours_org, contours_org, -1, (0,255,0), 3)
img_contours_pre = img_res.copy()
cv.drawContours(img_contours_pre, contours_pre, -1, (0,255,0), 3)
img_contours_post = img_res.copy()
cv.drawContours(img_contours_post, contours_post, -1, (0,255,0), 3)

#Code to display or save results
show = ""

if show == "single":
    cv.imshow("Display window", masked_img)
    k = cv.waitKey(0)

if show == "following":
    cv.imshow("Display window", img_res)
    k = cv.waitKey(0)
    cv.imshow("Display window", img_gray)
    k = cv.waitKey(0)
    cv.imshow("Display window", img_th1)
    k = cv.waitKey(0) 
    cv.imshow("Display window", img_open)
    k = cv.waitKey(0)
    cv.imshow("Display window", img_med)
    k = cv.waitKey(0)
    cv.imshow("Display window", img_gaus)
    cv.imshow("Display window2", img_contours_org)
    k = cv.waitKey(0)
    cv.imshow("Display window2", img_contours_pre)
    cv.imshow("Display window", img_rect_pre)
    k = cv.waitKey(0)
    cv.imshow("Display window2", img_contours_post)
    cv.imshow("Display window", img_rect_post)
    k = cv.waitKey(0)

if show == 'save':
    cv.imwrite("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/Results/OnSystem/gray.jpeg", img_gray)
    cv.imwrite("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/Results/OnSystem/threshold.jpeg", img_th1)
    cv.imwrite("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/Results/OnSystem/opening.jpeg", img_open)
    cv.imwrite("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/Results/OnSystem/median.jpeg", img_med)
    cv.imwrite("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/Results/OnSystem/gaus.jpeg", img_gaus)
    cv.imwrite("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/Results/OnSystem/contours_org.jpeg", img_contours_org)
    cv.imwrite("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/Results/OnSystem/contours_pre.jpeg", img_contours_pre)
    cv.imwrite("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/Results/OnSystem/rectangles_pre.jpeg", img_rect_pre)
    cv.imwrite("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/Results/OnSystem/contours_post.jpeg", img_contours_post)
    cv.imwrite("C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/test/Images/Results/OnSystem/rectangles_post.jpeg", img_rect_post)