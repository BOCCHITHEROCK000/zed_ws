import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import random
import time

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1초마다 콜백 호출

    def timer_callback(self):
        # Subscriber로부터 요청이 들어왔는지 확인하는 로직을 구현해야 하지만,
        # 단순화를 위해 여기서 매번 데이터를 발행하는 것으로 가정합니다.
        msg = Imu()

        # 임의의 IMU 데이터를 생성 (실제 사용에서는 센서 데이터로 대체)
        msg.linear_acceleration.x = random.uniform(-1.0, 1.0)
        msg.linear_acceleration.y = random.uniform(-1.0, 1.0)
        msg.linear_acceleration.z = random.uniform(-1.0, 1.0)
        msg.angular_velocity.x = random.uniform(-1.0, 1.0)
        msg.angular_velocity.y = random.uniform(-1.0, 1.0)
        msg.angular_velocity.z = random.uniform(-1.0, 1.0)

        self.publisher_.publish(msg)
        self.get_logger().info('Published IMU data')

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
