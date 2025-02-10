import rclpy, json, cv2, os, time
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

'''
ROS2 Node designed to run the YOLOv11 Segmentation CNN
may want to change the confidence level of the CNN in this file
or update the weights file in the weights directory if a more robust CNN is made.

The published data is a json encoded string in this format
{
    "0": [[100.5, 200.3], [150.8, 250.4]],
    "1": [[300.7, 400.9], [350.1, 450.2]]
}
'''

class YoloPredictionNode(Node):
    def __init__(self):
        super().__init__('yolo_prediction_node')
        #finds the weight file for the yolo CNN. if weights are updated they can be stored in the weights file,
        # change name in weights path variable and rebuild with colcon
        package_share_directory = get_package_share_directory('yolo_prediction')
        weights_path = os.path.join(package_share_directory, 'weights', 'YOLOv11_8.pt')

        #create a subscriber for the color image topic
        self.subscription = self.create_subscription(Image, '/processed/color_image', self.image_callback, 10)

        #create centroid publisher
        self.publisher_ = self.create_publisher(String, 'centroid_data', 10)

        #Initialize the YOLO model
        self.model = YOLO(weights_path) #change this if weights model is updated
        self.get_logger().info("YOLO model loaded from {weights_path}")

        # initilaise CvBridge for ROS to OpenCV conversions
        self.bridge = CvBridge()

        #creates a folder in whatever directory the node is run in to publish images with predictions
        self.output_dir = os.path.join(os.getcwd(), 'processed_images')
        os.makedirs(self.output_dir, exist_ok=True)

        #store last image
        self.latest_image = None

    #loads the image and if successful use the CNN to make a prediction
    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info("Recieved image for prediction")
            self.process_image_and_publish()
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def process_image_and_publish(self):
        if self.latest_image is None:
            self.get_logger().warning("No image available for prediction")
            return
        try:

                                                                   #play with confidence level to find the optimal amount
            #perform YOLO prediction                               #Field test showed it detecting dead leaves and knots on branches with 25%
            results = self.model.predict(source=self.latest_image, conf=0.25, save=False, show=False)

            #initialize dictionary to store centroids by class
            class_centroids = {0: [],  1: []}   #0 = calyx 1 = Kiwifruit

            colors = {0: (0, 255, 0), 1: (255, 0, 0)}#green for calyx #blue for kiwfruit
            
            #copy original image for visualising centroids
            output_image = self.latest_image.copy()

            # loop through the detected instances
            for result in results:
                if hasattr(result, 'masks') and result.masks is not None:   # check if masks are available
                    for mask, cls in zip(result.masks.xy, result.boxes.cls): # access masks and class
                        cls = int(cls)  #ensure class is integer
                        if cls in class_centroids:
                            #convert mask polygons to binary mask
                            mask_array = np.array(mask, dtype=np.int32) #ensure mask is numpy array
                            binary_mask = np.zeros_like(self.latest_image[:,:,0], dtype=np.uint8)   #create blank binary mask
                            cv2.fillPoly(binary_mask, [mask_array], 255) #fill the polygon on binary mask

                            #calculates the centroid of each detected mask
                            moments = cv2.moments(binary_mask)
                            if moments["m00"] != 0:
                                centroid_x = int(moments["m10"] / moments["m00"])
                                centroid_y = int(moments["m01"] / moments["m00"])
                                class_centroids[cls].append((centroid_x, centroid_y))

                                #visualize detected objects
                                cv2.circle(output_image, (centroid_x, centroid_y), 5, colors[cls], -1)
                                cv2.putText(output_image, f"Class {cls}", (centroid_x + 10, centroid_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors[cls], 1, cv2.LINE_AA)

            message_data = json.dumps(class_centroids)  #serialize the dictionary as a JSON string
            #create string message
            msg = String()
            msg.data = message_data
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published centroids: {message_data}")

            timestamp = time.strftime("%Y%m%d-%H%M%S")
            output_file = os.path.join(self.output_dir, f"processed_image_{timestamp}.png")
            cv2.imwrite(output_file, output_image)
            #self.get_logger().info(f"Saved processed image to {output_file}")

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloPredictionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("shutting Down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()