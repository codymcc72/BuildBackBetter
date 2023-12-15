#!/usr/bin/env python3

"""
TODO:
- Ideal payload runtimes
- Add linear velocity topic and use to check stops with gps points
- Ideal boom position
- plot robot path
- plot stops
- plot assists
- plot 
"""

import rosbag
import pandas as pd
import json
import math
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
from tric_navigation.msg import PLC_Feedback
from json_data_map import JsonDataMap  # Import the JsonDataMap class
import matplotlib.pyplot as plt


def load_json_data(file_path):
    print(f"Loading JSON data from: {file_path}")
    print("\n")
    try:
        with open(file_path, 'r') as file:
            return json.load(file)
    except Exception as e:
        print(f"Error loading JSON data: {str(e)}")
        return None

def load_rosbag(bag_path, topics):
    print(f"Loading rosbag data from: {bag_path}")
    print("\n")
    bag = rosbag.Bag(bag_path)
    messages = {topic: [] for topic in topics}

    for topic, msg, t in bag.read_messages(topics=topics):
        messages[topic].append(msg)

    bag.close()
    return messages

# Calculate haversine distance between two coordinates
def haversine_distance(lat1, lon1, lat2, lon2):
    # Haversine formula to calculate distance between two points on a sphere
    R = 6371000  # Earth radius in meters
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c

    return distance

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

# Extracts and plots the JSON map data from JsonDataMap class
def process_and_plot_map(json_map, json_data):
    # Extract and store data
    json_map.rows = json_map.extract_rows(json_data)
    json_map.turns = json_map.extract_turns(json_data)
    json_map.start_path = json_map.extract_start_path(json_data)
    json_map.end_path = json_map.extract_end_path(json_data)
    json_map.datum = json_map.extract_datum(json_data)

    # Check if turns data is available
    if json_map.turns:
        # Plot the data
        json_map.plot_data()
        #plt.show()

        # Calculate and return the total ideal time
        total_ideal_time = json_map.ideal_times()
        return total_ideal_time

    else:
        print("No turns data available to plot.")
        return 0  # Return 0 if there is no turns data

# Calculates the distance traveled with the payload on
def payload_runtime(df):
    # Convert the 'timestamp' column to datetime format
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

"""
For the Payload distance function I was only able to test it with the lights not on
the script below is what it should look like to calculate the distance of the payload
but I need to collect a rosbag with uvc_light_status messages before I will know if this works
"""

def payload_distance(df):
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
        distance = haversine_distance(lat1, lon1, lat2, lon2)

        # Accumulate the total distance
        total_distance += distance

    # Print the result
    print(f"Total distance traveled with Payload: {total_distance} meters")

def time_between_assists(df):
    # Find indices where 'manual_status' changes from True to False (assist ends)
    assist_end_indices = df.index[(df['joystick_control'] == True) & (df['joystick_control'].shift(-1) == False)].tolist()

    # Find indices where 'manual_status' changes from False to True (auto ends)
    auto_end_indices = df.index[(df['joystick_control'] == False) & (df['joystick_control'].shift(-1) == True)].tolist()

    # Ensure the lists have the same length
    min_len = min(len(assist_end_indices), len(auto_end_indices))
    assist_end_indices = assist_end_indices[:min_len]
    auto_end_indices = auto_end_indices[:min_len]

    # Calculate time differences between assists and autos in minutes (rounded to 2 decimals)
    time_between_assist_and_auto = [
        round((auto_timestamp - assist_timestamp).total_seconds() / 60, 2)
        for assist_timestamp, auto_timestamp in zip(df.loc[assist_end_indices, 'timestamp'], df.loc[auto_end_indices, 'timestamp'])
    ]

    return time_between_assist_and_auto

"""
The find stops funcrion calculates the differences between consecutive longitude (x) and latitude (y) values
Then identifies the indices where both differences are zero, indicating a stop.

- Add linear velocity checks
"""

def find_stops(df):
    # Calculate the differences in longitude (x) and latitude (y) between consecutive rows
    delta_x = df['longitude'].diff()
    delta_y = df['latitude'].diff()

    # Find the indices where both delta_x and delta_y are zero, indicating a stop
    stop_indices = df.index[(delta_x == 0) & (delta_y == 0)].tolist()

    return stop_indices

def runtime_comparison(df):
    # Convert 'timestamp' column to datetime format
    df['timestamp'] = pd.to_datetime(df['timestamp'], unit='s', utc=True)

    # Find the minimum and maximum timestamps
    start_time = df['timestamp'].min()
    end_time = df['timestamp'].max()

    # Calculate the total runtime
    total_runtime = end_time - start_time

    return total_runtime

def calculate_mode_times(df):
    # makes sure the DataFrame is not empty
    if df.empty:
        return 0, 0

    # Converts 'timestamp' column to datetime format
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
            #print(f"Manual Interval: {start} to {end}, Time Diff: {time_diff}, Accumulated Time Manual: {time_in_manual_minutes}")
        current_mode = not current_mode  # Switch mode

    # Reset mode flag
    current_mode = df['joystick_control'].iloc[0]

    # Calculate time in auto mode
    time_in_auto_minutes = 0
    for start, end in zip(auto_indices, auto_indices[1:]):
        if not current_mode:
            time_diff = (df.loc[end, 'timestamp'] - df.loc[start, 'timestamp']).total_seconds() / 60
            time_in_auto_minutes += time_diff
            #print(f"Auto Interval: {start} to {end}, Time Diff: {time_diff}, Accumulated Time Auto: {time_in_auto_minutes}")
        current_mode = not current_mode  # Switch mode

    return time_in_manual_minutes, time_in_auto_minutes

def start_position(df, json_map, threshold=0.00001):
    # Extract the first row from the DataFrame
    first_row = df.iloc[0]

    # Get the first x and y positions from the DataFrame
    first_x, first_y = first_row['longitude'], first_row['latitude']

    # Get the datum coordinates from the JSON map
    datum_x, datum_y = json_map.datum['x'], json_map.datum['y']

    # Calculate the distance between the first point in the DataFrame and the datum
    distance = haversine_distance(first_y, first_x, datum_y, datum_x)

    # Create a boolean mask for points within the threshold of the initial coordinates
    mask = df.apply(lambda row: (abs(row['latitude'] - first_y) <= threshold) and (abs(row['longitude'] - first_x) <= threshold), axis=1)

    # Filter the DataFrame based on the mask
    parked_df = df[mask]

    # Calculate the time spent at the initial coordinates
    if not parked_df.empty:
        time_parked = parked_df['timestamp'].max() - parked_df['timestamp'].min()
        print(f"Time Parked at initial coordinates: {time_parked}")
    else:
        print("Robot did not stay within the threshold at the initial coordinates.")

    print(f"Distance from the start position to the datum: {round(distance, 2)} meters")

def end_position(df, json_map):

    # Get the final x and y coordinates from the df
    last_x, last_y = df['longitude'].max(), df['latitude'].max()

    # Get the datum coordinates from the JSON map
    datum_x, datum_y = json_map.datum['x'], json_map.datum['y']

    # Calculate the distance between the first point in the DataFrame and the datum
    distance = haversine_distance(last_y, last_x, datum_y, datum_x)

    print(f"Distance from End Position to Datum: {round(distance, 2)} meters")

def print_summary(json_map, df, total_runtime_value, total_ideal_time, manual_time, auto_time, time_between_assist_and_auto, stop_indices):

    #Ideal Time Summary:

    #Distances Summary:
    print("\n")
    json_map.calculate_and_print_summary_distances()

    #Runtime Summary:
    print("\nRuntime Summary:")
    start_time = df['timestamp'].min()
    end_time = df['timestamp'].max()
    print(f"Start Time: {start_time.strftime('%Y-%m-%d %I:%M:%S %p')}")
    print(f"End Time: {end_time.strftime('%Y-%m-%d %I:%M:%S %p')}")
    print(f"Total Runtime: {total_runtime_value}")
    # Subtract total ideal time from total runtime
    difference = total_runtime_value.total_seconds() / 60 - total_ideal_time
    print(f"Difference between Total Runtime and Total Ideal Time: {round(difference, 2)} minutes")

    #Stops Summary
    print("\nStop Summary:")
    for stop_index in stop_indices:
        stop_duration = df.loc[stop_index, 'timestamp'] - df.loc[stop_index - 1, 'timestamp']
        print(f"Stop indices: {stop_index}, Latitude: {df.loc[stop_index, 'latitude']:.6f}, Longitude: {df.loc[stop_index, 'longitude']:.6f}, Duration: {stop_duration}")

    #Payload Summary
    print("\nPayload Summary:")
    payload_runtime(df)
    payload_distance(df)

    #Time Between Assists
    print("\nMTBA:")
    print(f"Time between assists: {time_between_assist_and_auto}")
    assist_length = len(time_between_assist_and_auto)
    print(f"Mean: {round(sum(time_between_assist_and_auto)/assist_length, 2)}")

    #Mode Summary
    print("\nMode Time Summary:")
    print(f"Time in Manual Mode: {round(manual_time, 2)} minutes")
    print(f"Time in Auto Mode: {round(auto_time, 2)} minutes")
    manual_vs_auto = (manual_time/auto_time)*100
    print(f"{round(manual_vs_auto, 2)}% of time was in manual")

    # Start Position Summary
    print("\nStart Position:")
    start_position(df, json_map)

    print("\nEnd Position:")
    end_position(df, json_map)

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

    df = create_dataframe(messages)

    # Define the path to JSON map data
    json_data_path = '/home/tric/codys_ws/src/data_collection/src/json_map_plot/testrow.json'

    # Create an instance of JsonDataMap
    json_map = JsonDataMap()

    # Load JSON data from file
    loaded_json_data = load_json_data(json_data_path)

    if loaded_json_data is not None:
        # Process and plot map segments, and capture the total ideal time
        total_ideal_time = process_and_plot_map(json_map, loaded_json_data)

        # Check if the total ideal time is calculated successfully
        if total_ideal_time > 0:

            # Initialize other variables needed for print_summary
            total_runtime_value = runtime_comparison(df)
            manual_time, auto_time = calculate_mode_times(df)
            time_between_assist_and_auto = time_between_assists(df)
            stop_indices = find_stops(df)

            # Print all summaries
            print_summary(json_map, df, total_runtime_value, total_ideal_time, manual_time, auto_time, time_between_assist_and_auto, stop_indices)

        else:
            print("Failed to calculate total ideal time. Exiting.")

    else:
        print("Failed to load JSON data. Exiting.")

# Call the main function
if __name__ == "__main__":
    main()
