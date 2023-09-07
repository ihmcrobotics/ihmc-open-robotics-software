import matplotlib.pyplot as plt
import numpy as np


if __name__ == "__main__":
    file = open('data/map_console_output_201947.txt', 'r')

    lines = file.readlines()

    data = np.zeros(shape=(len(lines), 8))

    fields = None

    for i,line in enumerate(lines):
        line = line.replace(' ', '')
        words = line.split(',')
        
        if len(words) == 8:

            if fields is None:
                fields = {word.split(':')[0] : k for k, word in enumerate(words)}
                print(fields)

            row = np.array([float(word.split(':')[1]) for word in words])
            data[i,:] = row

    time = np.arange(0, len(lines), 1)
    
    # plt.plot(time, data[:, fields['ExtractionTime']], label='Extraction Time')
    # plt.plot(time, data[:, fields['RegistrationTime']], label='Registration Time')
    # plt.plot(time, data[:, fields['MergingTime']], label='Merging Time')
    # plt.plot(time, data[:, fields['OptimizationTime']], label='Optimization Time')
    # plt.plot(time, data[:, fields['TotalTime']], label='Total Time')
    plt.plot(time, data[:, fields['Area']], label='Area')
    plt.plot(time, data[:, fields['Regions']], label='Regions')
    plt.plot(time, data[:, fields['Vertices']], label='Vertices')

    # Create subplots for each of these in a single pplot
    


    plt.legend()
    plt.show()
                
            