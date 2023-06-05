from ultralytics import YOLO

# Load a model
#model = YOLO("yolov8n.pt")  # load an official model
model = YOLO("weights.pt")  # load a custom model

# Predict with the model
results = model("../createDataset/dataset/imgs/0.jpg")  # predict on an image
print(results[0].numpy())
