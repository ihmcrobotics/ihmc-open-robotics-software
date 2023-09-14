import matplotlib.pyplot as plt
import numpy as np

def extract_data(filename):
    
    file = open(filename, 'r')

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

    data = np.vstack(data)

    return data, fields

def plot_data(label, data, fields, ax, fig, style='r-'):

    time = np.arange(0, len(data), 1)

    ax[0].plot(time, data[:, fields['Area']], style, label=label)
    ax[1].plot(time, data[:, fields['Regions']], style, label=label)
    ax[2].plot(time, data[:, fields['Vertices']], style, label=label)

    # plt.plot(time, data[:, fields['ExtractionTime']], label='Extraction Time')
    # plt.plot(time, data[:, fields['RegistrationTime']], label='Registration Time')
    # plt.plot(time, data[:, fields['MergingTime']], label='Merging Time')
    # plt.plot(time, data[:, fields['OptimizationTime']], label='Optimization Time')
    # plt.plot(time, data[:, fields['TotalTime']], label='Total Time')

    # plt.plot(time, data[:, fields['Area']], label='Area')
    # plt.plot(time, data[:, fields['Regions']], label='Regions')
    # plt.plot(time, data[:, fields['Vertices']], label='Vertices')

if __name__ == "__main__":
    
    files = [
        # ('data/map_console_output_tum_fr1_xyz.txt','r-'),
        # ('data/map_console_output_tum_fr3_office.txt', 'b-'),
        ('data/map_console_output_202456.txt', 'g-', 'E1'),
        ('data/map_console_output_201455.txt', 'y-', 'E2'),
        ('data/map_console_output_195802.txt', 'b-', 'E3'),
        ('data/map_console_output_201947.txt', 'r-', 'E4'),

    ]
    
    f, ax = plt.subplots(3, 1, figsize=(15,8))

    for file, style, label in files:

        ax[2].legend(loc='upper left')
        
        data, fields = extract_data(file)        
        
        plot_data(label, data, fields, ax, f, style)
    
    # Set X and Y labels
    ax[2].set_xlabel("Keyframe")

    ax[0].set_ylabel("Area")
    ax[1].set_ylabel("Regions")
    ax[2].set_ylabel("Vertices")

    # set legend
    ax[0].legend(loc='upper left')
    ax[1].legend(loc='upper left')

    plt.show()
                
            