from ultralytics import YOLO
import cv2
import numpy as np

# result.boxes.xyxy   # box with xyxy format, (N, 4)
# result.boxes.xywh   # box with xywh format, (N, 4)
# result.boxes.xyxyn  # box with xyxy format but normalized, (N, 4)
# result.boxes.xywhn  # box with xywh format but normalized, (N, 4)
# result.boxes.conf   # confidence score, (N, 1)
# result.boxes.cls    # cls, (N, 1)

"""
Matrix to store data for each block:

        class   confidence    box
block1    0       0.9999    (x, y)
block2    1       0.4678    (x, y)
block3    8       0.6534    (x, y)
"""

model = YOLO('best.pt')

results = model.predict(source='1.jpg', imgsz=1280, stream=True)

matrix = np.empty((0,4))

for result in results:

    blockClass = int(result.boxes.cls.tolist()[0])
    confidence = round(result.boxes.conf.tolist()[0],4)

    #print("Block: " + str(i) + "\n   Class: " + str(blockClass) + "\n   Confidence: " + str(confidence))

    x1 = int(result.boxes.xyxy[0][0])
    y1 = int(result.boxes.xyxy[0][1])
    x2 = int(result.boxes.xyxy[0][2])
    y2 = int(result.boxes.xyxy[0][3])

    #find the center of the square
    x = int((x1 + x2) / 2)
    y = int((y1 + y2) / 2)

    matrix = np.vstack((matrix, [blockClass, confidence, x, y]))
    #print(matrix)

img = cv2.imread('1.jpg')
for i in range(matrix.shape[0]):

    x = int(matrix[i][2])
    y = int(matrix[i][3])

    img = cv2.circle(img, (x, y), 5, (255, 0, 0), -1)

cv2.imshow('image', img)
cv2.waitKey(0)