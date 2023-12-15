import math
import matplotlib.pyplot as plt
import time

class JsonDataMap:
    def __init__(self):
        # Initialize attributes to store map segments
        self.rows = None
        self.turns = None
        self.start_path = None
        self.end_path = None
        self.datum = None
        
    # Extract rows data from JSON
    def extract_rows(self, json_data):
        x = [point['head']['position']['x'] + json_data['datum']['longitude'] for point in json_data['points']]
        y = [point['head']['position']['y'] + json_data['datum']['latitude'] for point in json_data['points']]

        # Extracting row points and adjusting their coordinates
        rows = [
            {
                'x': point['head']['position']['x'] + json_data['datum']['longitude'],
                'y': point['head']['position']['y'] + json_data['datum']['latitude']
            }
            for point in json_data['points'] if point.get('treatment_area', False)
        ]

        # Extracting x and y coordinates from the adjusted rows
        rows_x = [point['x'] for point in rows]
        rows_y = [point['y'] for point in rows]

        # Calculate total distance for rows
        total_distance = sum(math.sqrt((rows_x[i+1] - rows_x[i])**2 + (rows_y[i+1] - rows_y[i])**2) for i in range(len(rows_x)-1))

        return {'x': rows_x, 'y': rows_y, 'total_distance': total_distance}
    
    # Extract turns data from JSON
    def extract_turns(self, json_data):
        x = [point['head']['position']['x'] + json_data['datum']['longitude'] for point in json_data['points']]
        y = [point['head']['position']['y'] + json_data['datum']['latitude'] for point in json_data['points']]

        # Finding indices of treatment area true points
        treatment_area_indices = [i for i, point in enumerate(json_data['points']) if point.get('treatment_area', False)]

        # Adjusting the turn coordinates coordinates
        turns = [
            {
                'x': point['head']['position']['x'] + json_data['datum']['longitude'],
                'y': point['head']['position']['y'] + json_data['datum']['latitude']
            }
            for i in range(1, len(treatment_area_indices))  # Iterating from the second treatment area point
            for point in json_data['points'][treatment_area_indices[i - 1] + 1: treatment_area_indices[i]]
        ]

        turns_x = [point['x'] for point in turns]
        turns_y = [point['y'] for point in turns]

        # Calculate total distance for turns
        total_distance = sum(math.sqrt((turns_x[i+1] - turns_x[i])**2 + (turns_y[i+1] - turns_y[i])**2) for i in range(len(turns_x)-1))

        return {'x': turns_x, 'y': turns_y, 'total_distance': total_distance}

    # Extract start path data from JSON
    def extract_start_path(self, json_data):
        x = [point['head']['position']['x'] + json_data['datum']['longitude'] for point in json_data['points']]
        y = [point['head']['position']['y'] + json_data['datum']['latitude'] for point in json_data['points']]

        # Finding the index of the first treatment area point
        first_treatment_area_index = next((i for i, point in enumerate(json_data['points']) if point.get('treatment_area', False)), None)

        # Defining the range for the start path
        start_path_x = x[:first_treatment_area_index]
        start_path_y = y[:first_treatment_area_index]

        # Calculate total distance for start path
        total_distance = sum(math.sqrt((start_path_x[i+1] - start_path_x[i])**2 + (start_path_y[i+1] - start_path_y[i])**2) for i in range(len(start_path_x)-1))

        return {'x': start_path_x, 'y': start_path_y, 'total_distance': total_distance}

    # Extract end path data from JSON
    def extract_end_path(self, json_data):
        x = [point['head']['position']['x'] + json_data['datum']['longitude'] for point in json_data['points']]
        y = [point['head']['position']['y'] + json_data['datum']['latitude'] for point in json_data['points']]

        # Finding the index of the last point in the map
        last_point_index = len(json_data['points']) - 1

        # Finding the index of the last treatment area point
        last_treatment_area_index = next((i for i in range(last_point_index, -1, -1) if json_data['points'][i].get('treatment_area', False)), None)

        # Defining the x and y range for the end path
        end_path_x = x[last_treatment_area_index:]
        end_path_y = y[last_treatment_area_index:]

        # Calculate total distance for end path
        total_distance = sum(math.sqrt((end_path_x[i+1] - end_path_x[i])**2 + (end_path_y[i+1] - end_path_y[i])**2) for i in range(len(end_path_x)-1))

        return {'x': end_path_x, 'y': end_path_y, 'total_distance': total_distance}

    # Extract datum (home) coordinates from JSON
    def extract_datum(self, json):
        x = json['datum']['longitude']
        y = json['datum']['latitude']
        return {'x': x, 'y': y}


    def calculate_and_print_summary_distances(self):
        # Calculate total distance for each segment
        rows_distance = self.calculate_distance(self.rows)
        turns_distance = self.calculate_distance(self.turns)
        start_path_distance = self.calculate_distance(self.start_path)
        end_path_distance = self.calculate_distance(self.end_path)

        # Print summary distances
        print("Distances Summary:")
        print(f"Total Distance Rows: {round(rows_distance, 2)}")
        print(f"Total Distance Turns: {round(turns_distance, 2)}")
        print(f"Total Distance Start Path: {round(start_path_distance, 2)}")
        print(f"Total Distance End Path: {round(end_path_distance, 2)}")
        print(f"Total Distance: {round(rows_distance+end_path_distance+start_path_distance+turns_distance, 2)}")


    def calculate_distance(self, segment):

        x = segment['x']
        y = segment['y']

        # Calculate total distance for the segment
        total_distance = sum(
            math.sqrt((x[i + 1] - x[i]) ** 2 + (y[i + 1] - y[i]) ** 2)
            for i in range(len(x) - 1)
        )

        return total_distance


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

    def calculate_ideal_time(self, segment, speed):
        distance = self.calculate_distance(segment)
        time = distance / speed
        return time

    # Plot the extracted map data
    def plot_data(self):
        # Scatter plot for rows
        plt.scatter(self.rows['x'],
                    self.rows['y'],
                    color='mediumseagreen',
                    label='Rows',
                    s=10)

        # Scatter plot for turns
        plt.scatter(self.turns['x'],
                    self.turns['y'],
                    color='lightsteelblue',
                    label='Turns',
                    s=10)

        # Scatter plot for start path
        plt.scatter(self.start_path['x'],
                    self.start_path['y'],
                    color='steelblue',
                    label='Start Path',
                    s=10)

        # Scatter plot for end path
        plt.scatter(self.end_path['x'],
                    self.end_path['y'],
                    color='tomato',
                    label='End Path',
                    s=10)

        # Scatter plot for datum (home)
        plt.scatter(self.datum['x'],
                    self.datum['y'],
                    color='black',
                    marker='x',
                    label='Home')

        # Add labels
        plt.xlabel('X Label')
        plt.ylabel('Y Label')

        # Add legend
        plt.legend()
