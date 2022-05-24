import sys

from zivid_interfaces.srv import Capture
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class PublishingSubscriber(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.subscription_camera = self.create_subscription(
            Image,
            '/zivid/depth/image',
            self.zivid_image_callback,
            1)
        #self.current_image = np.zeros([1200,1944,4], dtype=np.uint8)

        timer_period = 5  # seconds
        self.timer = self.create_timer(
            timer_period, 
        self.capture3d) #sends gripper pos accorting to timer

        self.cli = self.create_client(Capture, 'zivid/capture')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Capture.Request()

    def capture3d(self):
        print("Capturing")
        node = rclpy.create_node("zivid_camera_client")

        capture_client = node.create_client(Capture, "/zivid/capture")
        while not capture_client.wait_for_service(timeout_sec=1.0):
            print("service not available, waiting again...")

        request = Capture.Request()
        future = capture_client.call_async(request)
        node.destroy_node()
        rclpy.shutdown()

    def zivid_image_callback(self, msg):
        self.current_image = msg.data
        print("Recieved image")
        self.use_image()

    def use_image(self):
        image_state = self.current_image
        print("Using image")


    def send_request(self):
        self.req 
        #halla = Capture.Request()
        #self.req.b = int(sys.argv[2])
        #self.future = self.cli.call_async(self.req)



def main(args=None):

    rclpy.init(args=args)

    publisher_and_subscriber = PublishingSubscriber()

    rclpy.spin(publisher_and_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher_and_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()