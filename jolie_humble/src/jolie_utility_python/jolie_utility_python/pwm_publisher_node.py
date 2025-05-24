import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from builtin_interfaces.msg import Time
import yaml
import math
import os
from ament_index_python.packages import get_package_share_directory

class PwmPublisherNode(Node):
    def __init__(self):
        super().__init__('pwm_publisher_node')
        self.declare_parameter('scenario_file', 'scenario.yaml')

        # Publishers
        self.pwm_pub = self.create_publisher(Float32, 'pwm_cmd', 10)
        self.time_pub = self.create_publisher(Time, 'current_time', 10)

        # Load YAML
        scenario_file = self.get_parameter('scenario_file').get_parameter_value().string_value
        scenario_path = os.path.join(
            get_package_share_directory('jolie_utility_python'),
            'config',
            scenario_file
        )
        with open(scenario_path, 'r') as f:
            config = yaml.safe_load(f)

        # Batas PWM
        self.min_pwm = config.get('min_pwm', -1.0)
        self.max_pwm = config.get('max_pwm', 1.0)
        # Daftar skenario
        self.scenarios = config.get('scenarios', [])

        # Hitung total durasi sebagai max(end) di semua skenario
        self.total_duration = 0.0
        for sc in self.scenarios:
            end_time = sc.get('end', 0.0)
            if end_time > self.total_duration:
                self.total_duration = end_time

        self.get_logger().info(f"Total loop duration: {self.total_duration:.3f} s")

        # Timer (100 Hz)
        self.start_time = self.get_clock().now().nanoseconds
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        # Waktu sekarang (detik) sejak start
        now_ros = self.get_clock().now()
        elapsed_ns = now_ros.nanoseconds - self.start_time
        now_sec = elapsed_ns * 1e-9

        # Loop time
        if self.total_duration > 0:
            t_loop = now_sec % self.total_duration
        else:
            t_loop = 0.0

        # Tentukan PWM berdasarkan t_loop
        pwm_value = 0.0
        for scenario in self.scenarios:
            start = scenario.get('start', 0.0)
            end   = scenario.get('end',   0.0)
            if start <= t_loop <= end:
                stype = scenario.get('type')
                if stype == 'step':
                    pwm_value = scenario.get('value', 0.0)

                elif stype == 'sinus':
                    freq = scenario.get('frequency', 1.0)
                    amp  = scenario.get('amplitude', 1.0)
                    pwm_value = amp * math.sin(2 * math.pi * freq * (t_loop - start))

                elif stype == 'ramp':
                    v0 = scenario.get('start_value', 0.0)
                    v1 = scenario.get('end_value',   0.0)
                    fraction = (t_loop - start) / max((end - start), 1e-6)
                    pwm_value = v0 + fraction * (v1 - v0)

                break  # hanya satu skenario aktif

        # Clamp ke [min_pwm, max_pwm]
        pwm_value = max(min(pwm_value, self.max_pwm), self.min_pwm)

        # Publish PWM
        pwm_msg = Float32(data=pwm_value)
        self.pwm_pub.publish(pwm_msg)

        # Publish timestamp
        sec, nsec = now_ros.seconds_nanoseconds()
        time_msg = Time(sec=sec, nanosec=nsec)
        self.time_pub.publish(time_msg)

    def destroy(self):
        self.timer.cancel()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PwmPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy()
        rclpy.shutdown()
