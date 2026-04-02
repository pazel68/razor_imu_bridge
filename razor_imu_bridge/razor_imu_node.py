import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import math

class RazorImuNode(Node):
    def __init__(self):
        super().__init__('razor_imu_node')
        
        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)

        # ประกาศ ROS Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        
        # ✅ แก้ไขคอมเมนต์: หน่วยเป็นมาตรฐาน ROS แล้ว
        # Offset ของ Accelerometer (หน่วย m/s^2)
        self.declare_parameter('accel_x_offset', 0.0)
        self.declare_parameter('accel_y_offset', 0.0)
        self.declare_parameter('accel_z_offset', 0.0)
        
        # Offset ของ Gyroscope (หน่วย rad/s)
        self.declare_parameter('gyro_x_offset', 0.0)
        self.declare_parameter('gyro_y_offset', 0.0)
        self.declare_parameter('gyro_z_offset', 0.0)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        try:
            self.serial_port = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f"เชื่อมต่อ IMU สำเร็จที่พอร์ต: {port}")
        except Exception as e:
            self.get_logger().error(f"ไม่สามารถเชื่อมต่อ IMU ได้: {e}")
            return

        self.timer = self.create_timer(0.005, self.read_and_publish)

    def read_and_publish(self):
        if self.serial_port.in_waiting > 0:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                parts = line.split(',')
                
                if len(parts) >= 7:
                    # ค่าดิบจากบอร์ด (ax, ay, az หน่วย g | gx, gy, gz หน่วย dps)
                    ax = float(parts[1])
                    ay = float(parts[2])
                    az = float(parts[3])
                    gx = float(parts[4])
                    gy = float(parts[5])
                    gz = float(parts[6])

                    # ดึงค่า Offset (หน่วย m/s^2 และ rad/s)
                    ax_off = self.get_parameter('accel_x_offset').value
                    ay_off = self.get_parameter('accel_y_offset').value
                    az_off = self.get_parameter('accel_z_offset').value
                    gx_off = self.get_parameter('gyro_x_offset').value
                    gy_off = self.get_parameter('gyro_y_offset').value
                    gz_off = self.get_parameter('gyro_z_offset').value

                    msg = Imu()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "imu_link"

                    # ✅ จุดที่แก้ไข: คูณแปลงหน่วยก่อน แล้วค่อยลบ Offset
                    # แปลงหน่วย Accelerometer (g -> m/s^2) แล้วหักลบ Offset
                    msg.linear_acceleration.x = (ax * 9.80665) - ax_off
                    msg.linear_acceleration.y = (ay * 9.80665) - ay_off
                    msg.linear_acceleration.z = (az * 9.80665) - az_off

                    # แปลงหน่วย Gyroscope (dps -> rad/s) แล้วหักลบ Offset
                    msg.angular_velocity.x = (gx * (math.pi / 180.0)) - gx_off
                    msg.angular_velocity.y = (gy * (math.pi / 180.0)) - gy_off
                    msg.angular_velocity.z = (gz * (math.pi / 180.0)) - gz_off

                    msg.orientation_covariance[0] = -1.0
                    self.publisher_.publish(msg)

            except ValueError:
                pass
            except Exception as e:
                self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RazorImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
