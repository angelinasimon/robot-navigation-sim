# import sys
# from ultralytics import YOLO

# image_path = sys.argv[1]

# model = YOLO("yolov8s.pt")
# results = model(frame, conf=0.25, verbose=False)

# for result in results:
#     for box in result.boxes:
#         class_id = int(box.cls[0])
#         class_name = model.names[class_id]
#         confidence = float(box.conf[0])
#         x1, y1, x2, y2 = box.xyxy[0].tolist()

#         print(
#             f"{class_name}: conf={confidence:.2f}, "
#             f"bbox=({x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f})"
#         )

#     result.save(filename="test_frame_yolo.jpg")

# print(f"Analyzed: {image_path}")
# print("Saved annotated image to test_frame_yolo.jpg")
