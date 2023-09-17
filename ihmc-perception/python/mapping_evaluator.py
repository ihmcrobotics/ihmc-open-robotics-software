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

def extract_plan_data(filename):
    file = open(filename, 'r')

    lines = file.readlines()
    data = []
    fields = None

    success = 0
    total = 0

    for i,line in enumerate(lines):

        if 'Result:' in line:

            total += 1

            content = line.split('):')[1]
            content = content.replace('\n', '')
            content = content.replace(' ', '')
            words = content.split(',')
            
            if len(words) == 5:

                if fields is None:
                    fields = {k : word.split(':')[0] for k, word in enumerate(words)}
                    print(fields)

                values = [float(words[i].split(':')[1]) for i in range(1, len(words))]

                if words[0].split(':')[1] == "FOUND_SOLUTION":
                    success += 1

                    # print(words)

                    row = np.array([values[i] for i in range(len(values))])

                    data.append(row)

    data = np.vstack(data)

    return data, fields, success, total


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

def map_stats_main():
    files = [
        ('data/map_console_output_tum_fr1_xyz.txt','r-'),
        ('data/map_console_output_tum_fr3_office.txt', 'b-'),
        # ('data/map_console_output_202456.txt', 'g-', 'E1'),
        # ('data/map_console_output_201455.txt', 'y-', 'E2'),
        # ('data/map_console_output_195802.txt', 'b-', 'E3'),
        # ('data/map_console_output_201947.txt', 'r-', 'E4'),

    ]
    
    # f, ax = plt.subplots(3, 1, figsize=(15,8))

    # for file, style, label in files:

    #     ax[2].legend(loc='upper left')
        
    #     data, fields = extract_data(file)        
        
    #     plot_data(label, data, fields, ax, f, style)
    
    # # Set X and Y labels
    # ax[2].set_xlabel("Keyframe")

    # ax[0].set_ylabel("Area")
    # ax[1].set_ylabel("Regions")
    # ax[2].set_ylabel("Vertices")

    # # set legend
    # ax[0].legend(loc='upper left')
    # ax[1].legend(loc='upper left')

    # plt.show()

    data, fields = extract_data('data/map_console_output_tum_fr3_office.txt')

    print("Final Size (MB) one-fifth: ", data[440, fields['Vertices']] * 12 / 1024 / 1024)
    print("Final Size (MB) two-fifth: ", data[880, fields['Vertices']] * 12 / 1024 / 1024)
    print("Final Size (MB) three-fifth: ", data[1320, fields['Vertices']] * 12 / 1024 / 1024)
    print("Final Size (MB) four-fifth: ", data[1760, fields['Vertices']] * 12 / 1024 / 1024)
    print("Final Size (MB) full: ", data[-1, fields['Vertices']] * 12 / 1024 / 1024)
    
    print("Average Total Time: ", np.mean(data[:, fields['TotalTime']]), " Std: ", np.std(data[:, fields['TotalTime']]))



def plan_stats_main():

    # titles = [  'WalkForward_FallAtTheEnd', # 20230228_191411_PerceptionLog.hdf5
    #         'Circular_Inward',          # 20230228_195802_PerceptionLog.hdf5
    #         'Circular_Turn',            # 20230228_195937_PerceptionLog.hdf5
    #         'Circular_Outward_Fall',    # 20230228_200243_PerceptionLog.hdf5
    #         'Circular_Outward',         # 20230228_201455_PerceptionLog.hdf5
    #         'WalkForward_Rough',        # 20230228_201947_PerceptionLog.hdf5
    #         'Turn_Rough',               # 20230228_202104_PerceptionLog.hdf5
    #         'WalkBack_Rough',           # 20230228_202456_PerceptionLog.hdf5
    #         'Stairs_ClimbUp',           # 20230228_204753_PerceptionLog.hdf5
    #         ]

    # plan_console_global_195802.txt  plan_console_global_201455.txt  plan_console_global_201947.txt  plan_console_global_202456.txt  plan_console_global_204753.txt  plan_console_local_195802.txt  plan_console_local_201455.txt  plan_console_local_201947.txt  plan_console_local_202456.txt  plan_console_local_204753.txt

    files = [
        ('data/plan/plan_console_local_201947.txt', 'Rough Forward (Local)'),
        ('data/plan/plan_console_global_201947.txt', 'Rough Forward'),
        ('data/plan/plan_console_global_back_201947.txt', 'Rough Forward (Back)'),

        ('data/plan/plan_console_local_202456.txt', 'Rough Backward (Local)'),
        ('data/plan/plan_console_global_202456.txt', 'Rough Backward'),
        ('data/plan/plan_console_global_back_202456.txt', 'Rough Backward (Back)'),



        ('data/plan/plan_console_local_195802.txt', 'Circular Inward (Local)'),
        ('data/plan/plan_console_global_195802.txt', 'Circular Inward'),
        ('data/plan/plan_console_global_back_195802.txt', 'Circular Inward (Back)'),

        ('data/plan/plan_console_local_201455.txt', 'Circular Outward (Local)'),
        ('data/plan/plan_console_global_201455.txt', 'Circular Outward'),
        ('data/plan/plan_console_global_back_201455.txt', 'Circular Outward (Back)'),

        # ('data/plan/plan_console_local_204753.txt', 'Stairs Climb Up (Local)'),
        # ('data/plan/plan_console_global_204753.txt', 'Stairs Climb Up'),
    ]

    for file, name in files:
        print("----------------------------------------------")
        print(name)


        data, fields, success, total = extract_plan_data(file)

        print("Success: ", success, " Total: ", total, " Success Rate: ", success/total)
        # print("Data Shape: ", data.shape)
        # print(fields)

        # print(fields[1], np.mean(data[:, 0]), np.std(data[:, 0]))
        print(fields[2], np.mean(data[:, 1]), np.std(data[:, 1]))
        # print(fields[3], np.mean(data[:, 2]),  np.std(data[:, 2]))
        # print(fields[4], np.mean(data[:, 3]), np.std(data[:, 3]))


if __name__ == "__main__":
    map_stats_main()

                
            