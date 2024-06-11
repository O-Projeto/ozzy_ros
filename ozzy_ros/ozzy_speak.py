import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time
from threading import Thread
from playsound import playsound

class MouthServoController(Node):

    def __init__(self, timings, audio_file):
        super().__init__('mouth_servo_controller')
        self.publisher_mouth = self.create_publisher(Int32, 'ozzy/servo/mouth', 10)
        self.publisher_neck_pan = self.create_publisher(Int32, 'ozzy/servo/neck/pan', 10)
        
        neck_msg = Int32()
        self.timings = timings
        self.index = 65
        self.state = False
        self.audio_file = audio_file
        self.get_logger().info('Mouth Servo Controller Node has been started.')
        self.timer = self.create_timer(0.1, self.timer_callback)  # Adjust the frequency as needed
        neck_msg.data = 0
        self.publisher_neck_pan.publish(neck_msg)
        time.sleep(1)
        neck_msg.data = 90
        self.publisher_neck_pan.publish(neck_msg)

        # Start the audio in a separate thread to play while moving the servo

        audio_thread = Thread(target=self.play_audio)
        audio_thread.start()

    def play_audio(self):
        playsound(self.audio_file)

    def timer_callback(self):
        if self.index < len(self.timings):
            msg = Int32()
            msg.data = 65 if self.state else 80 # Adjust as per your servo's range
            self.publisher_mouth.publish(msg)
            self.get_logger().info(f'Publishing Mouth Angle: {msg.data}')
            
            self.state = not self.state
            self.get_logger().info(f'Waiting for {self.timings[self.index]} milliseconds')
            time.sleep(self.timings[self.index]*2 / 1000.0)  # Sleep for the specified timing
            self.index += 1
        else:
            self.get_logger().info('Completed all timings. Stopping.')
            self.timer.cancel()  # Stop the timer

def main(args=None):
    rclpy.init(args=args)

    timings = [89.58246346555325, 115.17745302713989, 230.35490605427978, 
               179.1649269311065, 102.37995824634658, 127.97494780793322, 
               115.17745302713989, 204.75991649269315, 89.58246346555325, 
               179.1649269311065, 115.17745302713989, 230.35490605427978, 
               179.1649269311065, 102.37995824634658, 127.97494780793322, 
               115.17745302713989, 191.96242171189982, 89.58246346555325, 
               115.17745302713989, 204.75991649269315, 102.37995824634658, 
               191.96242171189982, 89.58246346555325, 127.97494780793322, 
               89.58246346555325, 115.17745302713989, 204.75991649269315, 
               89.58246346555325, 89.58246346555325, 179.1649269311065, 
               89.58246346555325, 102.37995824634658, 204.75991649269315, 
               89.58246346555325, 102.37995824634658, 204.75991649269315, 
               115.17745302713989, 102.37995824634658, 204.75991649269315, 
               89.58246346555325, 89.58246346555325, 191.96242171189982, 
               102.37995824634658, 191.96242171189982, 179.1649269311065, 
               115.17745302713989, 89.58246346555325, 63.98747390396661, 
               179.1649269311065, 102.37995824634658, 127.97494780793322, 
               179.1649269311065, 230.35490605427978, 89.58246346555325, 
               191.96242171189982, 89.58246346555325, 204.75991649269315, 
               115.17745302713989, 191.96242171189982, 179.1649269311065, 
               115.17745302713989, 63.98747390396661, 191.96242171189982, 
               89.58246346555325, 204.75991649269315, 115.17745302713989, 
               115.17745302713989, 179.1649269311065, 102.37995824634658, 
               89.58246346555325, 191.96242171189982, 102.37995824634658, 
               89.58246346555325, 204.75991649269315, 127.97494780793322, 
               127.97494780793322, 127.97494780793322, 204.75991649269315, 
               115.17745302713989, 179.1649269311065, 89.58246346555325, 
               89.58246346555325, 179.1649269311065, 179.1649269311065, 
               115.17745302713989, 179.1649269311065, 115.17745302713989, 
               102.37995824634658, 204.75991649269315, 115.17745302713989, 
               127.97494780793322, 89.58246346555325, 204.75991649269315, 
               89.58246346555325, 63.98747390396661, 179.1649269311065, 
               127.97494780793322, 204.75991649269315, 115.17745302713989, 
               179.1649269311065, 102.37995824634658, 204.75991649269315, 
               102.37995824634658, 89.58246346555325, 179.1649269311065, 
               127.97494780793322, 89.58246346555325, 89.58246346555325, 
               204.75991649269315, 89.58246346555325, 179.1649269311065, 
               89.58246346555325, 179.1649269311065, 89.58246346555325, 
               191.96242171189982, 115.17745302713989, 179.1649269311065, 
               127.97494780793322, 102.37995824634658, 191.96242171189982, 
               89.58246346555325, 127.97494780793322, 230.35490605427978, 
               127.97494780793322, 102.37995824634658, 204.75991649269315, 
               89.58246346555325, 204.75991649269315, 230.35490605427978, 
               89.58246346555325, 191.96242171189982, 89.58246346555325]
    
    audio_file = '/home/dino/ros2_ws/src/ozzy_ros/rcx.wav'
    node = MouthServoController(timings, audio_file)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
