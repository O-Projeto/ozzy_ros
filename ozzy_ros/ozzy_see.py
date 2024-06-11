import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge

class FaceDetectionNode(Node):
    def __init__(self):
        super().__init__('face_detection_node')
        self.image_pub = self.create_publisher(Image, '/face_detection/image', 10)
        self.face_pub = self.create_publisher(Int32MultiArray, '/face_detection/face_coordinates', 10)
        self.bridge = CvBridge()

    def detect_bounding_box(self, vid):
        gray_image = cv2.cvtColor(vid, cv2.COLOR_BGR2GRAY)
        faces = self.face_classifier.detectMultiScale(gray_image, 1.1, 5, minSize=(40, 40))
        for (x, y, w, h) in faces:
            cv2.rectangle(vid, (x, y), (x + w, y + h), (0, 255, 0), 4)
        return faces, vid

    def main_loop(self):
        self.face_classifier = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
        video_capture = cv2.VideoCapture(0)
        
        while True:
            result, video_frame = video_capture.read()  # read frames from the video
            if not result:
                break  # terminate the loop if the frame is not read successfully

            faces, processed_frame = self.detect_bounding_box(video_frame)

            # Publish the processed frame to ROS 2 topic
           
            image_msg = self.bridge.cv2_to_imgmsg(processed_frame, "bgr8")
            self.image_pub.publish(image_msg)

            # Publish face coordinates
            face_coords = Int32MultiArray()
            face_coords.data = [int(x) for face in faces for x in face[:2]]  # Convert values to int
            self.face_pub.publish(face_coords)
                
            # except CvBridgeError as e:
            #     self.get_logger().error(f'CvBridge Error: {e}')

            # cv2.imshow(
            #     "My Face Detection Project", processed_frame
            # )  # display the processed frame in a window named "My Face Detection Project"

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        video_capture.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectionNode()
    node.main_loop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
