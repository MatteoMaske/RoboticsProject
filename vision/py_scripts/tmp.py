import cv2
from ultralytics import YOLO

model = YOLO('best.pt')
results = model.predict(source='1.png', imgsz=1571)
print("******")
print(results)
print("******")
print(results[0])
print("******")
print(len(results[0]))

if not (len(results[0]) == 0):
    print("Dio cane")

# for i in range (len(results[0].boxes.xyxy)):
#     print(results[0].boxes.xyxy[i])

# img = cv2.imread('img.jpeg')
# img = cv2.circle(img, (922, 643), 5, (255, 0, 0), -1)
# cv2.imshow('img',img)
# cv2.waitKey(0)

# model = YOLO('best.pt')

# results = model.predict(source='1.jpg', imgsz=IMGSZ, stream=True)

# matrix = np.empty((0,4))

# for result in results:

#     blockClass = int(result.boxes.cls.tolist()[0])
#     confidence = round(result.boxes.conf.tolist()[0],4)

#     #print("Block: " + str(i) + "\n   Class: " + str(blockClass) + "\n   Confidence: " + str(confidence))

#     x1 = int(result.boxes.xyxy[0][0])
#     y1 = int(result.boxes.xyxy[0][1])
#     x2 = int(result.boxes.xyxy[0][2])
#     y2 = int(result.boxes.xyxy[0][3])

#     #find the center of the square
#     x = int((x1 + x2) / 2)
#     y = int((y1 + y2) / 2)

#     matrix = np.vstack((matrix, [blockClass, confidence, x, y]))
#     #print(matrix)

# img = cv2.imread('1.jpg')
# for i in range(matrix.shape[0]):

#     x = int(matrix[i][2])
#     y = int(matrix[i][3])

#     img = cv2.circle(img, (x, y), 5, (255, 0, 0), -1)

# cv2.imshow('image', img)
# cv2.waitKey(0)
############################################################################################