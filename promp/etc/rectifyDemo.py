# -*- coding: utf-8 -*-
"""
Created on Wed Aug  2 15:32:16 2023

@author: lpenco
"""

import csv
n = 10

def negate_columns_in_csv(input_file, output_file, column_indices):
    # Read and process the CSV file, saving the results to a new CSV file
    with open(input_file, 'r') as input_csvfile, open(output_file, 'w', newline='') as output_csvfile:
        reader = csv.reader(input_csvfile)
        writer = csv.writer(output_csvfile)

        for row in reader:
            # Create a copy of the row to avoid modifying the original row
            modified_row = row.copy()

            # Negate elements at specified column indices
            for index in column_indices:
                try:
                    value = float(modified_row[index])
                    modified_row[index] = str(-value)
                except ValueError:
                    print(f"Warning: Non-numeric value at row {reader.line_num}, column {index + 1}")

            # Write the modified row to the output CSV file
            writer.writerow(modified_row)

if __name__ == "__main__":
    input_csv_file = str(n) + ".csv"
    output_csv_file = "o" + str(n) + ".csv"
    indices_to_negate = [0, 1, 2, 3]  # Specify the column indices to negate here
    negate_columns_in_csv(input_csv_file, output_csv_file, indices_to_negate)
