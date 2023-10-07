import csv
import os
import numpy as np

# def read_specific_columns(input_file):
#     with open(input_file, 'r') as csvfile:
#         reader = csv.DictReader(csvfile, delimiter=';')
#         values = [(row['s_m'], row['x_m']) for row in reader]
#     return values[:10]


def read_specific_columns(input_file):
    with open(input_file, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=';')
        header = next(reader)  # Read the header row
        values = [(row[1], row[2], row[3]) for row in reader]
    return values[:10]  # Only return the first 10 lines

def global_to_body_fixed(x_g, y_g, psi, x_init, y_init):
    # Define the 2D rotation and translation matrix
    R = np.array([[np.cos(psi), -np.sin(psi), -1*x_init],
                [np.sin(psi), np.cos(psi), -1*y_init],
                [0, 0, 1]
                ])
    
    # Create a column vector of global coords [x, y]
    p_global = np.array([[x_g], [y_g], [1]])
    
    # Apply the transformation
    p_body = np.dot(R, p_global)
    
    # Extract the resulting x_b and y_b
    x_b, y_b = p_body[0, 0], p_body[1, 0]
    
    return x_b, y_b

def process_csv_data(data):
    for row in data:
        # delta_x = 10
        # delta_y = 5

        x = float(row[0])  # Assuming x_m is the first column
        y = float(row[1]) # Assuming y_m is the second column
        print(f'Global Coordinates: x = {x}, y = {y}')
        # x = x - delta_x
        # y = y - delta_y
        psi = float(row[2])  # Assuming psi_rad is the third column

        yaw = 0
        x_init= 0
        y_init = 5
        x_body, y_body = global_to_body_fixed(x, y, yaw, x_init, y_init)
        
        print(f'Body-fixed Coordinates: x_b = {x_body}, y_b = {y_body}')

input_file = os.path.expanduser('~/pair/src/local_planner/data/test_points.csv')
data_x_y_psi = read_specific_columns(input_file)
process_csv_data(data_x_y_psi)
