import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from perception_msgs.msg import ImageMessage

class YOLOPublisherNode(Node):
    def __init__(self):
        super().__init__('yolo_publisher_node')

        # Create a subscriber to receive Image messages
        self.subscription = self.create_subscription(
            ImageMessage,
            '/ihmc/zed2/left_color',
            self.image_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def image_callback(self, msg):
        # Convert Image message to OpenCV image

        print('Received Image message: {}'.format(msg))

    def detect(self):
        
        # Create a new YOLO model from scratch
        model = YOLO('yolov8n.yaml')

        # Load a pretrained YOLO model (recommended for training)
        model = YOLO('yolov8n.pt')

        # Perform object detection on an image using the model
        results = model('https://ultralytics.com/images/bus.jpg')

def main(args=None):
    rclpy.init(args=args)
    node = YOLOPublisherNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()