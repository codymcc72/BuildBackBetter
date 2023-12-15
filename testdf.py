def load_rosbag(bag_path, topics):
    print(f"Loading rosbag data from: {bag_path}")
    print("\n")
    bag = rosbag.Bag(bag_path)
    messages = {topic: [] for topic in topics}

    for topic, msg, t in bag.read_messages(topics=topics):
        messages[topic].append(msg)

    bag.close()
    return messages

# Populates pandas df with the rosbag message information
def create_dataframe(messages):
    data = {'longitude': [], 'latitude': [], 'timestamp': [], 'joystick_control': [], 'uvc_light_status': [], 'boom_position': []}

    for gps_msg, joystick_msg, uvc_light_msg, plc_feedback_msg in zip(
        messages['/tric_navigation/gps/head_data'],
        messages['/tric_navigation/joystick_control'],
        messages['/tric_navigation/uvc_light_status'],
        messages['/tric_navigation/plc_feedback']
    ):
        data['longitude'].append(gps_msg.longitude)
        data['latitude'].append(gps_msg.latitude)
        data['timestamp'].append(gps_msg.header.stamp.to_sec())
        data['joystick_control'].append(joystick_msg.data)
        data['uvc_light_status'].append(uvc_light_msg.data)
        data['boom_position'].append(plc_feedback_msg.boom_position)

    df = pd.DataFrame(data)
    return df


def main():
    bag_path = '/home/tric/codys_ws/src/data_collection/src/e0_rosbags/2023-12-06-15-32-37.bag'
    topics = [
        '/tric_navigation/gps/head_data',
        '/tric_navigation/joystick_control',
        '/tric_navigation/uvc_light_status',
        '/tric_navigation/plc_feedback'
    ]

    messages = load_rosbag(bag_path, topics)

    if not all(messages[topic] for topic in topics):
        print(f"No messages found on one or more topics in the bag.")
        return
