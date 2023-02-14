from ultralytics import YOLO

# Load a model
model = YOLO("config.yaml")  # build a new model from scratch
model = YOLO("yolov8n.pt")  # load a pretrained model (recommended for training)
model.to("cpu")

# Use the model
model.train(data="data.yaml", epochs=20)  # train the model
metrics = model.val()  # evaluate model performance on the validation set
success = model.export()  # export the model
