import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.model = YOLO("src/self_driving/model/yolov8n.pt")
        self.tracker = DeepSort(max_age=30)
        self.track_classes = {}

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(frame)

        detections = []
        for result in results[0].boxes:
            confidence = result.conf[0]
            if confidence < 0.2:
                continue
            x1, y1, x2, y2 = map(int, result.xyxy[0])
            w, h = x2 - x1, y2 - y1
            class_index = int(result.cls[0])
            detections.append([[x1, y1, w, h], confidence, class_index])

        tracks = self.tracker.update_tracks(detections, frame=frame)
        for track in tracks:
            if not track.is_confirmed() or track.time_since_update > 1:
                continue
            bbox = track.to_tlbr()
            track_id = track.track_id
            class_id = track.get_det_class()
            class_name = self.model.names[class_id] if class_id != -1 else "Unknown"
            cv2.rectangle(frame, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (0, 255, 0), 2)
            label = f"{class_name} {track_id}"
            (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (int(bbox[0]), int(bbox[1]) - text_height - baseline), (int(bbox[0]) + text_width, int(bbox[1])), (0, 255, 0), -1)
            cv2.putText(frame, label, (int(bbox[0]), int(bbox[1]) - baseline), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

        cv2.imshow("YOLOv8 + Deep SORT Inference", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
