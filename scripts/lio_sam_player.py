import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import sys
from collections import deque
import time

class LioSamFlowControl(Node):
    def __init__(self, bag_path):
        super().__init__('lio_sam_flow_control')
        
        self.lidar_topic = '/velodyne_points'
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100
        )
        
        # 監聽後端里程計作為時間推進的依據
        self.sub_odom = self.create_subscription(
            Odometry, 
            'lio_sam/mapping/odometry', 
            self.odom_callback, 
            qos_profile
        )
        
        self.pub_clock = self.create_publisher(Clock, '/clock', 10)
        self.last_clock_ns = 0
        
        self.publishers_dict = {}
        self.exclude_topics = ['/clock']
        
        # 流控變數 (時間戳機制)
        self.lidar_queue = deque()
        self.max_queue_size = 30       # 允許 Python 端多讀幾張
        self.latest_backend_time = 0.0 # 後端處理到的最新 Bag Time
        self.max_time_ahead = 1.5      # 允許超前 1.5 秒 (防跳幀卡死)
        
        self.last_activity_time = time.time()
        
        self.last_wall_time_for_rate = time.time()
        self.last_bag_ns_for_rate = 0
        self.is_first_msg = True
        
        # 改為每 2 秒印出一次，更快看到回饋
        self.rate_timer = self.create_timer(2.0, self.print_playback_rate)
        
        self.reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.reader.open(storage_options, converter_options)
        
        topic_types = self.reader.get_all_topics_and_types()
        self.type_map = {t.name: t.type for t in topic_types}
        
        self.timer = self.create_timer(0.001, self.pump_messages)
        self.get_logger().info(f"播放器啟動 (時間戳流控最終版): {bag_path}")

    def odom_callback(self, msg):
        # 記錄後端處理完的時間戳
        self.latest_backend_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.last_activity_time = time.time()

    def print_playback_rate(self):
        if self.is_first_msg:
            return 
            
        current_wall_time = time.time()
        wall_dt = current_wall_time - self.last_wall_time_for_rate
        bag_dt_sec = (self.last_clock_ns - self.last_bag_ns_for_rate) / 1e9
        
        if wall_dt > 0 and bag_dt_sec > 0:
            rate = bag_dt_sec / wall_dt
            self.get_logger().info(
                f"⏱️ 近期播放倍率: {rate:.2f}x "
                f"(Bag 推進 {bag_dt_sec:.2f}s / 實際耗時 {wall_dt:.2f}s)"
            )
        
        self.last_wall_time_for_rate = current_wall_time
        self.last_bag_ns_for_rate = self.last_clock_ns

    def pump_messages(self):
        # [機制 1] 超時解鎖 (系統真的卡住超過 2.5 秒才介入)
        if (time.time() - self.last_activity_time) > 2.5 and self.latest_backend_time > 0:
            self.get_logger().warn("⚠️ 後端無回應超過 2.5 秒，強制推進時間解鎖！")
            self.latest_backend_time += 0.5 
            self.last_activity_time = time.time()

        # [機制 2] 處理暫存區的點雲
        while self.lidar_queue:
            next_msg = self.lidar_queue[0]
            next_msg_time = next_msg.header.stamp.sec + next_msg.header.stamp.nanosec / 1e9
            
            # --- 關鍵修正：防止開局暴衝 ---
            if self.latest_backend_time == 0:
                self.latest_backend_time = next_msg_time
                
            # 檢查時間差：如果領先超過設定值，就暫停發送
            if (next_msg_time - self.latest_backend_time) > self.max_time_ahead:
                break
                
            msg = self.lidar_queue.popleft()
            self.publishers_dict[self.lidar_topic].publish(msg)
            self.last_activity_time = time.time()

        # [機制 3] 從 Bag 讀取資料
        while len(self.lidar_queue) < self.max_queue_size and self.reader.has_next():
            (topic, data, t) = self.reader.read_next()
            
            if topic not in self.type_map or topic in self.exclude_topics or topic.startswith('/lio_sam/'):
                continue
                
            msg_type = get_message(self.type_map[topic])
            msg = deserialize_message(data, msg_type)
            
            if hasattr(msg, 'header'):
                msg_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                
                if self.is_first_msg:
                    self.last_bag_ns_for_rate = msg_ns
                    self.last_wall_time_for_rate = time.time()
                    self.is_first_msg = False
                    
                if msg_ns >= self.last_clock_ns:
                    self.last_clock_ns = msg_ns
                    clock_msg = Clock()
                    clock_msg.clock = msg.header.stamp
                    self.pub_clock.publish(clock_msg)

            if topic not in self.publishers_dict:
                if topic == '/tf_static':
                    custom_qos = QoSProfile(
                        reliability=QoSReliabilityPolicy.RELIABLE,
                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                        history=QoSHistoryPolicy.KEEP_LAST,
                        depth=10
                    )
                else:
                    custom_qos = 100
                self.publishers_dict[topic] = self.create_publisher(msg_type, topic, custom_qos)

            # 分流邏輯
            if topic == self.lidar_topic:
                self.lidar_queue.append(msg)
                next_msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                
                # --- 關鍵修正：防止開局暴衝 ---
                if self.latest_backend_time == 0:
                    self.latest_backend_time = next_msg_time
                    
                if (next_msg_time - self.latest_backend_time) <= self.max_time_ahead:
                    send_msg = self.lidar_queue.popleft()
                    self.publishers_dict[topic].publish(send_msg)
                    self.last_activity_time = time.time()
            else:
                self.publishers_dict[topic].publish(msg)
                
        # 結束判定
        if not self.reader.has_next() and len(self.lidar_queue) == 0:
            if (time.time() - self.last_activity_time) > 2.0:
                self.get_logger().info("✅ Bag 播放結束！")
                self.timer.cancel()
                self.rate_timer.cancel()
                raise SystemExit

def main(args=None):
    if len(sys.argv) < 2:
        print("用法: python3 lio_sam_player.py <bag_directory_path>")
        return

    rclpy.init(args=args)
    bag_path = sys.argv[1]
    
    try:
        player = LioSamFlowControl(bag_path)
        rclpy.spin(player)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
