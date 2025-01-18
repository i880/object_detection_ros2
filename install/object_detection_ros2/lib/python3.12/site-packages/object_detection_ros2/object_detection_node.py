#!/usr/bin/env python3
import rclpy  # Import rclpy for ROS 2 Python API
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # Import the String message type
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.bridge = CvBridge()

        # Get the path to the models directory
        package_share_directory = get_package_share_directory('object_detection_ros2')
        model_path = os.path.join(package_share_directory, 'models')

        # Load YOLO model
        self.net = cv2.dnn.readNet(
            os.path.join(model_path, 'yolov3-tiny.weights'),
            os.path.join(model_path, 'yolov3-tiny.cfg')
        )
        with open(os.path.join(model_path, 'coco.names'), "r") as f:
            self.classes = f.read().strip().split("\n")
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]

        # Subscriber for the camera image
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # Publisher for the detected object name
        self.object_name_pub = self.create_publisher(String, '/detected_object_name', 10)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width, channels = frame.shape

            # Detecting objects
            blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
            self.net.setInput(blob)
            outs = self.net.forward(self.output_layers)

            # Process detections
            class_ids = []
            confidences = []
            boxes = []
            for out in outs:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > 0.5:
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)

                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)

                        boxes.append([x, y, w, h])
                        confidences.append(float(confidence))
                        class_ids.append(class_id)

            indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
            for i in range(len(boxes)):
                if i in indexes:
                    x, y, w, h = boxes[i]
                    label = str(self.classes[class_ids[i]])
                    confidence = confidences[i]

                    # Publish the detected object name and confidence score
                    object_name_msg = String()
                    object_name_msg.data = f"{label} ({confidence:.2f})"  # Include confidence score
                    self.object_name_pub.publish(object_name_msg)

                    # Draw bounding box and label on the frame
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame, f"{label} {confidence:.2f}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("Object Detection", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

    def __del__(self):
        # Close OpenCV window when the node is destroyed
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
