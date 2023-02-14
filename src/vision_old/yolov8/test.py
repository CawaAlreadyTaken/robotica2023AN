from ultralytics import YOLO

# Load a model
#model = YOLO("yolov8n.pt")  # load an official model
model = YOLO("best.pt")  # load a custom model

# Predict with the model
results = model("1001.jpg")  # predict on an image
print(results[0].numpy())