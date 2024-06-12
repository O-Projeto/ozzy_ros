import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray

def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min
class NeckServoPublisher(Node):



    def __init__(self):
        super().__init__('neck_servo_publisher')
        self.publisher_left = self.create_publisher(Int32, 'ozzy/servo/neck/left', 5)
        self.publisher_right = self.create_publisher(Int32, 'ozzy/servo/neck/right', 5)
        self.publisher_pan = self.create_publisher(Int32, 'ozzy/servo/neck/pan', 5)
        self.subscriber = self.create_subscription(Int32MultiArray, 'face_detection/face_coordinates', self.face_coords_callback, 10)
        self.angle = 70
        self.increment = 1
        self.get_logger().info('Neck Servo Publisher Node has been started.')

    
    def ozzy(self):
        msg_left = Int32()
        msg_right = Int32()
        
        msg_left.data = self.angle
        msg_right.data = self.angle
        
        self.publisher_left.publish(msg_left)
        self.publisher_right.publish(msg_right)
        
        # self.get_logger().info(f'Publishing Left Neck Angle: {msg_left.data}')
        # self.get_logger().info(f'Publishing Right Neck Angle: {msg_right.data}')
        
        self.angle += self.increment
        if self.angle >= 120 or self.angle <= 70:
            self.increment *= -1

    def face_coords_callback(self, msg):
        msg_pan = Int32()
        servo_pan = 90
        # Assume the first two integers in the Int32MultiArray message are x and y coordinates
        if msg.data:
            x, y = msg.data[:2]
            servo_pan = map_range(x,0,400,0,180)   
        msg_pan.data = servo_pan

        self.get_logger().info(f'Publishing PAN Neck Angle: {msg_pan.data}|')
        self.publisher_pan.publish(msg_pan)
           

def main(args=None):
    rclpy.init(args=args)
    node = NeckServoPublisher()
    
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(6)

    try: 
        while rclpy.ok(): 
            # node.ozzy()
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()
