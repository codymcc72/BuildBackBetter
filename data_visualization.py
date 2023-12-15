import json
import math
import matplotlib.pyplot as plt
import pandas as pd
import rosbag

class JSONProcessor:
    COLOR_ROWS = 'green'
    COLOR_TURNS = 'yellow'
    COLOR_START_PATH = 'blue'
    COLOR_END_PATH = 'red'

    def __init__(self, json_file_path):
        with open(json_file_path, "r") as json_file:
            self.json_data = json.load(json_file)

        self.validate_json_structure()

        self.coordinates_df = self.extract_rows()
        self.turns_df = self.extract_turns()
        self.start_path_df = self.extract_start_path()
        self.end_path_df = self.extract_end_path()

    def validate_json_structure(self):
        if 'points' not in self.json_data or 'datum' not in self.json_data:
            raise ValueError("Invalid JSON structure. Missing 'points' or 'datum'.")

    def extract_rows(self):
        rows = [
            {
                'x': point['head']['position']['x'] + self.json_data['datum']['longitude'],
                'y': point['head']['position']['y'] + self.json_data['datum']['latitude']
            }
            for point in self.json_data['points'] if point.get('treatment_area', False)
        ]

        return pd.DataFrame(rows)

    def extract_turns(self):
        treatment_area_indices = [i for i, point in enumerate(self.json_data['points']) if point.get('treatment_area', False)]

        turns = [
            {
                'x': point['head']['position']['x'] + self.json_data['datum']['longitude'],
                'y': point['head']['position']['y'] + self.json_data['datum']['latitude']
            }
            for i in range(1, len(treatment_area_indices))
            for point in self.json_data['points'][treatment_area_indices[i - 1] + 1: treatment_area_indices[i]]
        ]

        return pd.DataFrame(turns)

    def extract_start_path(self):
        first_treatment_area_index = next((i for i, point in enumerate(self.json_data['points']) if point.get('treatment_area', False)), None)

        start_path = {
            'x': [point['head']['position']['x'] + self.json_data['datum']['longitude'] for point in self.json_data['points'][:first_treatment_area_index + 1]],
            'y': [point['head']['position']['y'] + self.json_data['datum']['latitude'] for point in self.json_data['points'][:first_treatment_area_index + 1]]
        }

        return pd.DataFrame(start_path)

    def extract_end_path(self):
        last_point_index = len(self.json_data['points']) - 1
        last_treatment_area_index = next((i for i in range(last_point_index, -1, -1) if self.json_data['points'][i].get('treatment_area', False)), None)

        end_path = {
            'x': [point['head']['position']['x'] + self.json_data['datum']['longitude'] for point in self.json_data['points'][last_treatment_area_index + 1:]],
            'y': [point['head']['position']['y'] + self.json_data['datum']['latitude'] for point in self.json_data['points'][last_treatment_area_index + 1:]]
        }

        return pd.DataFrame(end_path)

    def plot_points(self, df, color, label):
        plt.scatter(df['x'], df['y'], c=color, label=label)

    def plot_dataframes(self, *dataframes):
        # Plotting all DataFrames in one plot
        fig, ax = plt.subplots(figsize=(8, 8))

        for df, color, label in zip(dataframes, [self.COLOR_ROWS, self.COLOR_TURNS, self.COLOR_START_PATH, self.COLOR_END_PATH], ['Rows', 'Turns', 'Start Path', 'End Path']):
            if df is not None:
                ax.scatter(df['x'], df['y'], c=color, label=label)

        ax.set_title('Combined Plot of DataFrames')
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.legend()

        # Show the plot
        plt.show()

class ROSBagProcessor:
    def __init__(self, bag_path, topics):
        self.bag_path = bag_path
        self.topics = topics
        self.messages = self.load_rosbag()

    def load_rosbag(self):
        print(f"Loading rosbag data from: {self.bag_path}")
        print("\n")
        bag = rosbag.Bag(self.bag_path)
        messages = {topic: [] for topic in self.topics}

        for topic, msg, t in bag.read_messages(topics=self.topics):
            messages[topic].append(msg)

        bag.close()
        return messages

    def create_dataframe(self):
        data = {'longitude': [], 'latitude': [], 'timestamp': [], 'joystick_control': [], 'uvc_light_status': [], 'boom_position': []}

        for gps_msg, joystick_msg, uvc_light_msg, plc_feedback_msg in zip(
            self.messages['/tric_navigation/gps/head_data'],
            self.messages['/tric_navigation/joystick_control'],
            self.messages['/tric_navigation/uvc_light_status'],
            self.messages['/tric_navigation/plc_feedback']
        ):
            data['longitude'].append(gps_msg.longitude)
            data['latitude'].append(gps_msg.latitude)
            data['timestamp'].append(gps_msg.header.stamp.to_sec())
            data['joystick_control'].append(joystick_msg.data)
            data['uvc_light_status'].append(uvc_light_msg.data)
            data['boom_position'].append(plc_feedback_msg.boom_position)

        df = pd.DataFrame(data)
        return df
    

class DataLogger:
    def __init__(self, json_processor, rosbag_processor, rows, turns, start_path, end_path):
        self.json_processor = json_processor
        self.rosbag_processor = rosbag_processor
        self.rows = rows
        self.turns = turns
        self.start_path = start_path
        self.end_path = end_path
        self.total_ideal_time = None

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        # Haversine formula to calculate distance between two points on a sphere
        R = 6371000  # Earth radius in meters
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c

        return distance
    
    def total_runtime(self):
        # Access the DataFrame directly
        df_gps = self.rosbag_processor.create_dataframe()

        if 'timestamp' not in df_gps.columns:
            raise ValueError("DataFrame does not contain 'timestamp' column.")

        # Convert 'timestamp' column to datetime format
        df_gps['timestamp'] = pd.to_datetime(df_gps['timestamp'], unit='s', utc=True)

        # Find the minimum and maximum timestamps
        start_time = df_gps['timestamp'].min()
        end_time = df_gps['timestamp'].max()

        # Calculate the total runtime
        total_runtime = (end_time - start_time).total_seconds()

        return total_runtime

    
    def calculate_segment_distances(self, df):
        """
        Calculate the distance for each segment in the DataFrame.
        Assumes the DataFrame has 'x' and 'y' columns.
        """
        if 'x' not in df.columns or 'y' not in df.columns:
            raise ValueError("DataFrame must have 'x' and 'y' columns.")

        x_values = df['x'].values
        y_values = df['y'].values

        segment_distances = [math.sqrt((x_values[i + 1] - x_values[i]) ** 2 + (y_values[i + 1] - y_values[i]) ** 2)
                             for i in range(len(x_values) - 1)]

        total_distance = sum(segment_distances)

        return total_distance, segment_distances

    def calculate_json_map_distances(self):
        """
        Calculate distances for each segment of the JSON map.
        Returns a dictionary with total distances and segment distances for rows, turns, start path, and end path.
        """
        rows_distance, rows_segment_distances = self.calculate_segment_distances(self.json_processor.coordinates_df)
        turns_distance, turns_segment_distances = self.calculate_segment_distances(self.json_processor.turns_df)
        start_path_distance, start_path_segment_distances = self.calculate_segment_distances(self.json_processor.start_path_df)
        end_path_distance, end_path_segment_distances = self.calculate_segment_distances(self.json_processor.end_path_df)

        distances = {
            'total_distance_rows': rows_distance,
            'total_distance_turns': turns_distance,
            'total_distance_start_path': start_path_distance,
            'total_distance_end_path': end_path_distance,
           #'segment_distances_rows': rows_segment_distances,
           #'segment_distances_turns': turns_segment_distances,
           #'segment_distances_start_path': start_path_segment_distances,
           #'segment_distances_end_path': end_path_segment_distances,
        }

        return distances
    
    def calculate_mode_times(self):
        # Access the DataFrame directly
        df = self.rosbag_processor.create_dataframe()

        # Ensure the DataFrame is not empty
        if df.empty:
            return 0, 0

        # Convert 'timestamp' column to datetime format
        df['timestamp'] = pd.to_datetime(df['timestamp'], unit='s')

        # Find indices where 'joystick_control' is True (manual mode)
        manual_indices = df.index[df['joystick_control']].tolist()

        # Find indices where 'joystick_control' is False (auto mode)
        auto_indices = df.index[~df['joystick_control']].tolist()

        # Initialize mode flag
        current_mode = df['joystick_control'].iloc[0]

        # Calculate time in manual mode
        time_in_manual_minutes = 0
        for start, end in zip(manual_indices, manual_indices[1:]):
            if current_mode:
                time_diff = (df.loc[end, 'timestamp'] - df.loc[start, 'timestamp']).total_seconds() / 60
                time_in_manual_minutes += time_diff
            current_mode = not current_mode  # Switch mode

        # Reset mode flag
        current_mode = df['joystick_control'].iloc[0]

        # Calculate time in auto mode
        time_in_auto_minutes = 0
        for start, end in zip(auto_indices, auto_indices[1:]):
            if not current_mode:
                time_diff = (df.loc[end, 'timestamp'] - df.loc[start, 'timestamp']).total_seconds() / 60
                time_in_auto_minutes += time_diff
            current_mode = not current_mode  # Switch mode

        return time_in_manual_minutes, time_in_auto_minutes
    
    def ideal_times(self, treatment_speed=0.8, non_treatment_speed=0.8):
        # Calculate ideal travel times for each segment based on the provided speeds

        # Calculate ideal times for rows
        rows_ideal_time = self.calculate_ideal_time(self.rows, treatment_speed)

        # Calculate ideal times for turns
        turns_ideal_time = self.calculate_ideal_time(self.turns, non_treatment_speed)

        # Calculate ideal times for start path
        start_path_ideal_time = self.calculate_ideal_time(self.start_path, non_treatment_speed)

        # Calculate ideal times for end path
        end_path_ideal_time = self.calculate_ideal_time(self.end_path, non_treatment_speed)

        # Print ideal times
        print("Ideal Travel Times Summary:")
        print(f"Ideal Time Rows: {round(rows_ideal_time, 2)} seconds")
        print(f"Ideal Time Turns: {round(turns_ideal_time, 2)} seconds")
        print(f"Ideal Time Start Path: {round(start_path_ideal_time, 2)} seconds")
        print(f"Ideal Time End Path: {round(end_path_ideal_time, 2)} seconds")
        self.total_ideal_time = round(rows_ideal_time + turns_ideal_time + start_path_ideal_time + end_path_ideal_time) / 60
        print(f"Total Ideal Time: {round(self.total_ideal_time, 2)} minutes")

        return self.total_ideal_time

    def calculate_distance(self, df):
        """
        Calculate the distance for each segment in the DataFrame.
        Assumes the DataFrame has 'x' and 'y' columns.
        """
        if 'x' not in df.columns or 'y' not in df.columns:
            raise ValueError("DataFrame must have 'x' and 'y' columns.")

        x_values = df['x'].values
        y_values = df['y'].values

        segment_distances = [math.sqrt((x_values[i + 1] - x_values[i]) ** 2 + (y_values[i + 1] - y_values[i]) ** 2)
                             for i in range(len(x_values) - 1)]

        total_distance = sum(segment_distances)

        return total_distance

    def calculate_ideal_time(self, segment, speed):
        distance = self.calculate_distance(segment)
        time = distance / speed
        return time
    
    def find_stops(self):
        # Access the DataFrame directly
        df = self.rosbag_processor.create_dataframe()

        # Ensure the DataFrame is not empty
        if df.empty:
            return []

        # Calculate the differences in longitude (x) and latitude (y) between consecutive rows
        delta_x = df['longitude'].diff()
        delta_y = df['latitude'].diff()

        # Find the indices where both delta_x and delta_y are zero, indicating a stop
        stop_indices = df.index[(delta_x == 0) & (delta_y == 0)].tolist()

        # Print details about each stop
        for stop_index in stop_indices:
            stop_duration = df.loc[stop_index, 'timestamp'] - df.loc[stop_index - 1, 'timestamp']
            print(f"Stop Index: {stop_index}, Latitude: {df.loc[stop_index, 'latitude']:.6f}, Longitude: {df.loc[stop_index, 'longitude']:.6f}, Duration: {stop_duration}")

        return stop_indices
    
    def payload_runtime(self):
        # Convert the 'timestamp' column to datetime format
        df = self.rosbag_processor.create_dataframe()

        df['timestamp'] = pd.to_datetime(df['timestamp'], unit='s')

        # Filter rows where uvc_light_status is '000' (lights off)
        lights_off_data = df[df['uvc_light_status'] == '000']

        # Filter rows where uvc_light_status is '111' (lights on)
        lights_on_data = df[df['uvc_light_status'] == '111']

        # Calculate the total time lights were off
        total_lights_off_time = lights_off_data['timestamp'].diff().sum()

        # Calculate the total time lights were on
        total_lights_on_time = lights_on_data['timestamp'].diff().sum()

        # Print the results
        print(f"Total time lights were off: {total_lights_off_time}")
        print(f"Total time lights were on: {total_lights_on_time}")
    
    def payload_distance(self):
        # Access the DataFrame directly
        df = self.rosbag_processor.create_dataframe()

        # Ensure the DataFrame is not empty
        if df.empty:
            print("DataFrame is empty. Cannot calculate payload distance.")
            return

        # Convert the 'timestamp' column to datetime format
        df['timestamp'] = pd.to_datetime(df['timestamp'], unit='s')

        # Filter rows where uvc_light_status is '111' (lights on)
        lights_on_data = df[df['uvc_light_status'] == '111']

        # Calculate the distance traveled when lights were on
        total_distance = 0.0

        # Iterate through consecutive rows with lights on to calculate distance
        for i in range(len(lights_on_data) - 1):
            lat1, lon1 = lights_on_data.iloc[i]['latitude'], lights_on_data.iloc[i]['longitude']
            lat2, lon2 = lights_on_data.iloc[i + 1]['latitude'], lights_on_data.iloc[i + 1]['longitude']

            # Calculate distance between consecutive rows
            distance = self.haversine_distance(lat1, lon1, lat2, lon2)

            # Accumulate the total distance
            total_distance += distance

        # Print the result
        print(f"Total distance traveled with Payload: {total_distance} meters")

        return total_distance
    
    def time_between_assists(self):
        df = self.rosbag_processor.create_dataframe()

        if df.empty:
            print("DataFrame is empty. Cannot calculate time between assists.")
            return []

        df['timestamp'] = pd.to_datetime(df['timestamp'], unit='s')

        assist_end_indices = df.index[(df['joystick_control'] == True) & (df['joystick_control'].shift(-1) == False)].tolist()
        auto_end_indices = df.index[(df['joystick_control'] == False) & (df['joystick_control'].shift(-1) == True)].tolist()

        if not assist_end_indices or not auto_end_indices:
            print("No assists or autos found.")
            return []

        min_len = min(len(assist_end_indices), len(auto_end_indices))
        assist_end_indices = assist_end_indices[:min_len]
        auto_end_indices = auto_end_indices[:min_len]

        time_between_assist_and_auto = [
            round((auto_timestamp - assist_timestamp).total_seconds() / 60, 2)
            for assist_timestamp, auto_timestamp in zip(df.loc[assist_end_indices, 'timestamp'], df.loc[auto_end_indices, 'timestamp'])
        ]

        return time_between_assist_and_auto
    

def main():
    # JSONProcessor
    json_processor = JSONProcessor("testrow.json")
    rows = json_processor.extract_rows()
    turns = json_processor.extract_turns()
    start_path = json_processor.extract_start_path()
    end_path = json_processor.extract_end_path()

    # Plotting JSONProcessor DataFrames
    json_processor.plot_dataframes(rows, turns, start_path, end_path)

    # ROSBagProcessor
    bag_path = '2023-12-06-15-32-37.bag'
    topics = [
        '/tric_navigation/gps/head_data',
        '/tric_navigation/joystick_control',
        '/tric_navigation/uvc_light_status',
        '/tric_navigation/plc_feedback'
    ]

    rosbag_processor = ROSBagProcessor(bag_path, topics)
    rosbag_df = rosbag_processor.create_dataframe()

    if not all(rosbag_processor.messages[topic] for topic in topics):
        print(f"No messages found on one or more topics in the bag.")
        return

    # Example usage of ROSBagProcessor dataframe
   #print("ROSBag DataFrame:")
   #print(rosbag_df)

    # DataLogger
    data_logger = DataLogger(json_processor, rosbag_processor, rows, turns, start_path, end_path)

    # Print total runtime
    total_runtime = data_logger.total_runtime()
    print(f"Total Runtime: {total_runtime} seconds")

    # Print distances
    distances = data_logger.calculate_json_map_distances()
    for key, value in distances.items():
        print(f"{key}: {value}")

    # Print ideal times
    data_logger.ideal_times()

    # Calculate and print mode times
    manual_time, auto_time = data_logger.calculate_mode_times()
    print(f"Time in Manual Mode: {round(manual_time, 2)} minutes")
    print(f"Time in Auto Mode: {round(auto_time, 2)} minutes")

    # Find and print stop indices
    stop_indices = data_logger.find_stops()

    data_logger.payload_distance()
    data_logger.payload_runtime()

        # Call the time_between_assists method
    time_between_assist_and_auto = data_logger.time_between_assists()

    # Check if the result is not empty
    if time_between_assist_and_auto:
        assist_length = len(time_between_assist_and_auto)
        print(f"Time between assists: {time_between_assist_and_auto}")
        print(f"Mean: {round(sum(time_between_assist_and_auto) / assist_length, 2)}")
    else:
        print("No data available for time between assists.")
        
if __name__ == "__main__":
    main()
