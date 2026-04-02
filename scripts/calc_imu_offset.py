import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def calculate_imu_offsets(bag_folder_path, topic_name='/imu/data_raw'):
    print(f"กำลังอ่านข้อมูลจากโฟลเดอร์: {bag_folder_path} ...")
    print(f"Topic ที่ค้นหา: {topic_name}")
    
    # ตั้งค่าตัวอ่าน Bag ของ ROS2 ดั้งเดิม
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_folder_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"❌ เปิดไฟล์ Bag ไม่ได้: {e}")
        return

    # ค้นหาชนิดของ Message ของ Topic นี้
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    
    if topic_name not in type_map:
        print(f"❌ ไม่พบ Topic '{topic_name}' ใน Bag นี้")
        return
        
    # ดึงคลาส Message มาสำหรับ Decode ข้อมูล
    msg_type_str = type_map[topic_name]
    msg_type = get_message(msg_type_str)

    accel_x, accel_y, accel_z = [], [], []
    gyro_x, gyro_y, gyro_z = [], [], []

    print("กำลังประมวลผล...")
    
    # วนลูปอ่านข้อมูลทีละบรรทัด
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        if topic == topic_name:
            # แปลงข้อมูลดิบกลับมาเป็น Message
            msg = deserialize_message(data, msg_type)
            
            accel_x.append(msg.linear_acceleration.x)
            accel_y.append(msg.linear_acceleration.y)
            accel_z.append(msg.linear_acceleration.z)

            gyro_x.append(msg.angular_velocity.x)
            gyro_y.append(msg.angular_velocity.y)
            gyro_z.append(msg.angular_velocity.z)

    total_samples = len(accel_x)
    if total_samples == 0:
        print("❌ ไม่พบข้อมูล IMU ในไฟล์เลย")
        return

    # คำนวณหาค่าเฉลี่ย (Mean)
    mean_ax = sum(accel_x) / total_samples
    mean_ay = sum(accel_y) / total_samples
    mean_az = sum(accel_z) / total_samples

    mean_gx = sum(gyro_x) / total_samples
    mean_gy = sum(gyro_y) / total_samples
    mean_gz = sum(gyro_z) / total_samples

    # คำนวณ Offset (โดยแกน Z ของ Accel ต้องหักลบแรงโน้มถ่วง 9.80665 m/s^2 ออก)
    offset_ax = mean_ax
    offset_ay = mean_ay
    offset_az = mean_az - 9.80665

    offset_gx = mean_gx
    offset_gy = mean_gy
    offset_gz = mean_gz

    # พิมพ์ผลลัพธ์พร้อมใช้งาน
    print("\n" + "="*45)
    print("✅ สรุปผลการ Calibrate (Static Offset)")
    print("="*45)
    print(f"จำนวนข้อมูลที่นำมาหาค่าเฉลี่ย: {total_samples} samples")
    
    print("\n🎯 นำค่าพารามิเตอร์เหล่านี้ไปใส่ตอนรัน Node ได้เลย:")
    print(f"accel_x_offset:={offset_ax:.6f}")
    print(f"accel_y_offset:={offset_ay:.6f}")
    print(f"accel_z_offset:={offset_az:.6f}")
    print(f"gyro_x_offset:={offset_gx:.6f}")
    print(f"gyro_y_offset:={offset_gy:.6f}")
    print(f"gyro_z_offset:={offset_gz:.6f}")
    print("="*45 + "\n")

if __name__ == '__main__':
    # เช็คว่าผู้ใช้ใส่ชื่อโฟลเดอร์ Bag มาด้วยหรือไม่
    if len(sys.argv) < 2:
        print("วิธีใช้งาน: python3 calc_imu_offset.py <ชื่อโฟลเดอร์_bag>")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    calculate_imu_offsets(bag_path)
