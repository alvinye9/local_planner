import os

input_file = os.path.expanduser('~/IAC_ws/trajoutput.csv')

if os.path.exists(input_file):
    print(f"The file {input_file} exists.")
else:
    print(f"The file {input_file} does not exist.")