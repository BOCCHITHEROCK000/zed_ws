import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time

class IMUSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.listener_callback,
            10)
        self.subscription  # 방지: 'subscription' 변수의 할당을 무시
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1초마다 요청을 보내는 타이머

    def listener_callback(self, msg):
        accel = msg.linear_acceleration
        angular = msg.angular_velocity

        # 현재 시간
        current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())

        # 데이터 저장
        with open("src/imu_data_logger/imu_data.txt", "a") as f:
            f.write(f"Time: {current_time}\n")
            f.write(f"Acceleration: X={accel.x}, Y={accel.y}, Z={accel.z}\n")
            f.write(f"Angular Velocity: X={angular.x}, Y={angular.y}, Z={angular.z}\n")
            f.write("\n")

        self.get_logger().info('IMU data received and saved')

    def timer_callback(self):
        # 실제 구현에서는 Publisher에게 데이터를 요청하는 로직이 포함되어야 합니다.
        # 하지만, 예제의 단순화를 위해 1초마다 자동으로 데이터를 받는다고 가정합니다.
        pass

def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = IMUSubscriber()
    rclpy.spin(imu_subscriber)
    imu_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
