from ultralytics import YOLO

# Load a model
#model = YOLO("yolov8n.pt")  # load an official model
model = YOLO("best.pt")  # load a custom model

# Predict with the model
results = model("imgs/3594.jpg")  # predict on an image
print(type(results[0]))
print(results[0].boxes.data)