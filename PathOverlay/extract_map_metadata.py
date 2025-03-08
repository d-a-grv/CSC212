from pathlib import Path
from rosbags.highlevel import AnyReader

bag_path = Path(r'C:\Users\Danii\Documents\rosbag2_2025_03_07-11_33_28')  # adjust as needed

with AnyReader([bag_path]) as reader:
    # Filter for the map topic
    connections = [conn for conn in reader.connections if conn.topic == '/map']
    for connection, timestamp, rawdata in reader.messages(connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        # Extract the map metadata from msg.info
        resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height
        origin = msg.info.origin  # This is a Pose message.
        print("Map Resolution (m/pixel):", resolution)
        print("Map Dimensions (pixels):", width, "x", height)
        print("Map Origin:")
        print("  Position:", origin.position.x, origin.position.y, origin.position.z)
        print("  Orientation:", origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w)
        break  # exit after processing the first map message
