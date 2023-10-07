import csv
import matplotlib.pyplot as plt
import os

def plot_csv(file_path):
    x_values = []
    y_values = []

    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file, delimiter=';')  # Set the delimiter to ';'
        next(csv_reader)  # Skip the header row if it exists

        for row in csv_reader:
            x_values.append(float(row[1]))  # Assuming 0-indexed columns
            y_values.append(float(row[2]))

    plt.scatter(x_values, y_values, label='Data Points')
    plt.xlabel('Column 2')
    plt.ylabel('Column 3')
    plt.title('Scatter Plot of Column 2 vs. Column 3')
    plt.legend()
    plt.show()

# Replace 'your_file.csv' with the actual path of your CSV file

input_file = os.path.expanduser('~/pair/src/local_planner/data/test_points.csv')
plot_csv(input_file)
