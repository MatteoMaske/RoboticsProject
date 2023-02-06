import cv2

img = cv2.imread('img.jpeg')
img = cv2.circle(img, (922, 643), 5, (255, 0, 0), -1)
cv2.imshow('img',img)
cv2.waitKey(0)