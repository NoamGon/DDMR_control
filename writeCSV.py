import csv

def write_to_csv(file_name, data, mode='a'):
    with open(file_name, mode, newline='') as file:
        writer = csv.writer(file)
        writer.writerow(data)