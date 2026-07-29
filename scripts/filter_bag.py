import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from tf2_msgs.msg import TFMessage

def filter_specific_tf(input_bag, output_bag):
    # 1. 初始化 Reader
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=input_bag, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', 
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)
    
    # 2. 初始化 Writer
    writer = rosbag2_py.SequentialWriter()
    out_storage_options = rosbag2_py.StorageOptions(uri=output_bag, storage_id='sqlite3')
    writer.open(out_storage_options, converter_options)
    
    # 3. 複製所有 Topic 的 Metadata
    for topic_meta in reader.get_all_topics_and_types():
        writer.create_topic(topic_meta)
        
    print(f"🚀 開始將 {input_bag} 過濾至 {output_bag}...")
    print("🔍 正在掃描並移除包含 'map' 與 'base_link' 之間的靜態轉換...")
    
    count = 0
    removed_count = 0
    
    # 4. 迴圈讀取並過濾
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        
        if topic == '/tf_static':
            # 將二進位資料解碼成 TFMessage 結構
            msg = deserialize_message(data, TFMessage)
            
            # 準備一個新的陣列來裝「合法」的 TF
            filtered_transforms = []
            
            for tf in msg.transforms:
                # 判斷這筆轉換是不是我們要殺掉的目標
                # (同時檢查 map->base_link 與 base_link->map 的可能性)
                is_bad_tf = (tf.header.frame_id == 'map' and tf.child_frame_id == 'base_link') or \
                            (tf.header.frame_id == 'base_link' and tf.child_frame_id == 'map')
                
                if is_bad_tf:
                    removed_count += 1
                    continue  # 跳過它，不加入新陣列
                
                # 其他正常的 TF 就加進來
                filtered_transforms.append(tf)
            
            # 如果剔除壞東西後，裡面還有其他的靜態 TF，就把它包裝好寫入
            if len(filtered_transforms) > 0:
                msg.transforms = filtered_transforms
                new_data = serialize_message(msg)
                writer.write(topic, new_data, timestamp)
                
        else:
            # 其他非 /tf_static 的 Topic 直接原封不動寫入
            writer.write(topic, data, timestamp)
            
        count += 1
        if count % 20000 == 0:
            print(f"已處理 {count} 筆訊息...")
            
    print(f"✅ 過濾完成！成功移除了 {removed_count} 筆 'map' <-> 'base_link' 的靜態轉換。")

if __name__ == '__main__':
    # ⚠️ 請替換成你實際的資料夾路徑
    INPUT_BAG_PATH = '/home/spie/bag/' 
    OUTPUT_BAG_PATH = '/home/spie/bag/'
    
    filter_specific_tf(INPUT_BAG_PATH, OUTPUT_BAG_PATH)