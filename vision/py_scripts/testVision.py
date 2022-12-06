import torch

weight_path = 'Users/Robotics/RoboticsProject/vision/data/weights/yolov5s.pt'




model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True, weights=weight_path)
