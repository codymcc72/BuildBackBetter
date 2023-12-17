import rosbag
import pandas as pd
import matplotlib.pyplot as plt
import math
from math import radians, sin, cos, sqrt, atan2

class GPSDataProcessor:
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
        data = {'longitude': [], 'latitude': [], 'timestamp': []}

        # Find the start time of the recording
        start_time = pd.to_datetime(self.messages['/tric_navigation/gps/head_data'][0].header.stamp.to_sec(), unit='s')

        for gps_msg in self.messages['/tric_navigation/gps/head_data']:
            data['longitude'].append(gps_msg.longitude)
            data['latitude'].append(gps_msg.latitude)

            # Calculate timestamp relative to the recording start, rounded to seconds with 2 decimal places
            timestamp_relative = pd.to_datetime(gps_msg.header.stamp.to_sec(), unit='s') - start_time
            timestamp_seconds = timestamp_relative.total_seconds()
            data['timestamp'].append(timestamp_seconds)

        df = pd.DataFrame(data)
        return df

class JoystickDataProcessor:
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

    def create_dataframe(self, assumed_frequency=12.3):
        data = {'joystick_control': [], 'timestamp': []}

        # Assuming messages are sorted by timestamp
        time_since_start = 0  # Initialize time_since_start to 0

        for joystick_msg in self.messages['/tric_navigation/joystick_control']:
            data['joystick_control'].append(joystick_msg.data)

            # Calculate timestamp based on the assumed frequency, rounded to seconds with 2 decimal places
            timestamp_seconds = time_since_start
            data['timestamp'].append(timestamp_seconds)
            time_since_start += 1 / assumed_frequency  # Increase time_since_start by the reciprocal of the assumed frequency

        df = pd.DataFrame(data)
        return df
    
class DataLogger:

    @staticmethod
    def merge_dataframes(gps_df, joystick_df):
        # Merge dataframes
        merged_df = pd.merge_asof(joystick_df, gps_df, on='timestamp', direction='nearest')

        # Calculate the differences in longitude (x) and latitude (y) between consecutive rows
        delta_x = merged_df['longitude'].diff()
        delta_y = merged_df['latitude'].diff()

        # Create a mask for rows where both longitude and latitude don't change
        mask = (delta_x == 0) & (delta_y == 0)

        # Remove rows where both longitude and latitude don't change
        merged_df = merged_df[~mask]

        # Reset the index to have a continuous sequence
        merged_df = merged_df.reset_index(drop=True)

        return merged_df
    
    def calculate_distances(self, df):
        # Calculate distances between consecutive GPS coordinates using the Haversine formula
        distances = []
        for i in range(1, len(df)):
            lat1, lon1 = radians(df.loc[i - 1, 'latitude']), radians(df.loc[i - 1, 'longitude'])
            lat2, lon2 = radians(df.loc[i, 'latitude']), radians(df.loc[i, 'longitude'])

            dlat = lat2 - lat1
            dlon = lon2 - lon1

            a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
            c = 2 * atan2(sqrt(a), sqrt(1 - a))

            # Radius of the Earth in meters (you can adjust this if needed)
            radius_earth = 6371000

            distance = radius_earth * c
            distances.append(distance)

        return distances

    def calculate_distance_traveled(self, df):
        # Use the calculate_distances method to get distances between consecutive points
        distances = self.calculate_distances(df)

        # Sum up the distances to get the total distance traveled
        total_distance_traveled = sum(distances)

        return total_distance_traveled
    
    def total_runtime(self, merged_df):
        # Access the DataFrame directly
        df_gps = merged_df

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
    
    def time_between_assists(self, merged_df):
        df = merged_df.copy()

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
    
    def calculate_mode_times(self, merged_df):
        df = merged_df.copy()

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
    
    def find_stops(self, merged_df):
        df = merged_df.copy()

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
    
    def find_stops(self, merged_df, threshold_meters=(.2)/1111):
        df = merged_df.copy()

        # Ensure the DataFrame is not empty
        if df.empty:
            return []

        # Calculate the distances between consecutive GPS coordinates using the Haversine formula
        distances = self.calculate_distances(df)

        # Find the indices where any distance is less than or equal to the threshold
        stop_indices = [i for i, distance in enumerate(distances) if distance <= threshold_meters]

        # Print details about each stop
        for stop_index in stop_indices:
            if stop_index > 0:  # Skip the first index to avoid accessing an invalid index
                stop_duration = df.loc[stop_index, 'timestamp'] - df.loc[stop_index - 1, 'timestamp']
                print(f"Stop Index: {stop_index}, Latitude: {df.loc[stop_index, 'latitude']:.6f}, Longitude: {df.loc[stop_index, 'longitude']:.6f}, Duration: {stop_duration}")

        return stop_indices
    
def main():
    bag_file_path_gps = "2023-12-06-14-46-37.bag"
    bag_file_path_joystick = "2023-12-06-14-46-37.bag"

    topics_to_process_gps = ['/tric_navigation/gps/head_data']
    topics_to_process_joystick = ['/tric_navigation/joystick_control']

    gps_processor = GPSDataProcessor(bag_file_path_gps, topics_to_process_gps)
    joystick_processor = JoystickDataProcessor(bag_file_path_joystick, topics_to_process_joystick)

    df_gps = gps_processor.create_dataframe()
    df_joystick = joystick_processor.create_dataframe()

    # Call the merge_dataframes method from the DataLogger class
    merged_df = DataLogger.merge_dataframes(df_gps, df_joystick)

    pd.set_option('display.max_columns', None)
    pd.set_option('display.max_rows', None)
    pd.set_option('display.width', None)
    pd.options.display.float_format = '{:.15f}'.format  # Display the full float value
    print(merged_df)

    # Call the total_runtime method from the DataLogger class
    total_runtime = DataLogger().total_runtime(merged_df)
    print(f'Total Runtime: {total_runtime} seconds')

    # Call the time_between_assists method from the DataLogger class
    time_between_assists = DataLogger().time_between_assists(merged_df)
    print(f'Time Between Assists: {time_between_assists} minutes')

    # Call the calculate_mode_times method from the DataLogger class
    time_in_manual, time_in_auto = DataLogger().calculate_mode_times(merged_df)
    print(f'Time in Manual Mode: {time_in_manual} minutes')
    print(f'Time in Auto Mode: {time_in_auto} minutes')

    # Call the find_stops method from the DataLogger class
    stop_indices = DataLogger().find_stops(merged_df)
    print(f'Stop Indices: {stop_indices}')

    # Call the calculate_distance_traveled method from the DataLogger class
    distance_traveled = DataLogger().calculate_distance_traveled(merged_df)
    print(f'Distance Traveled: {distance_traveled} meters')

if __name__ == "__main__":
    main()
