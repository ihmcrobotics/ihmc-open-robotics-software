import matplotlib.pyplot as plt
import numpy as np


if __name__ == "__main__":
    file = open('data/map_console_output_201455.txt', 'r')

    lines = file.readlines()

    

    data = []

    fields = None

    for i,line in enumerate(lines):
        line = line.replace(' ', '')
        words = line.split(',')
        
        if len(words) == 8:

            if fields is None:
                fields = {word.split(':')[0] : k for k, word in enumerate(words)}
                print(fields)

            row = np.array([float(word.split(':')[1]) for word in words])
            data.append(row)

    time = np.arange(0, len(data), 1)
    data = np.vstack(data)
    
    f, ax = plt.subplots(3, 1, figsize=(15,8))
    
    ax[0].set_title("Area")
    ax[1].set_title("Regions")
    ax[2].set_title("Vertices")

    ax[0].plot(time, data[:, fields['Area']], 'r-')
    ax[1].plot(time, data[:, fields['Regions']], 'r-')
    ax[2].plot(time, data[:, fields['Vertices']], 'r-')

    # plt.plot(time, data[:, fields['ExtractionTime']], label='Extraction Time')
    # plt.plot(time, data[:, fields['RegistrationTime']], label='Registration Time')
    # plt.plot(time, data[:, fields['MergingTime']], label='Merging Time')
    # plt.plot(time, data[:, fields['OptimizationTime']], label='Optimization Time')
    # plt.plot(time, data[:, fields['TotalTime']], label='Total Time')
    # plt.plot(time, data[:, fields['Area']], label='Area')
    # plt.plot(time, data[:, fields['Regions']], label='Regions')
    # plt.plot(time, data[:, fields['Vertices']], label='Vertices')

    # Create subplots for each of these in a single pplot
    


    plt.legend()
    plt.show()
                
            