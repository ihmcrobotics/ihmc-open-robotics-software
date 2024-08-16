from footstep_dataset_visualizer import visualize_plan
from hdf5_reader import *
from terrain_map_analyzer import plot_and_compute_stats

def generate_footstep_sequences(grid_size):
    sequences = []
    for x1 in range(grid_size):
        for y1 in range(grid_size):
            for x2 in range(grid_size):
                for y2 in range(grid_size):
                    for x3 in range(grid_size):
                        for y3 in range(grid_size):
                            sequence = [(x1, y1), (x2, y2), (x3, y3)]
                            sequences.append(sequence)
    return sequences

def run_footstep_generator(height_map):

    # Example usage
    grid_size = 5
    sequences = generate_footstep_sequences(grid_size)
    print(sequences)
    def is_valid_sequence(sequence):
        # Add your validation logic here
        # Return True if the sequence is valid, False otherwise
        return True

    valid_sequences = []
    for sequence in sequences:
        if is_valid_sequence(sequence):
            valid_sequences.append(sequence)

    print(valid_sequences)


if __name__ == "__main__":
    # Load the data
    home_path = os.path.expanduser("~")
    data = h5py.File(home_path + "/Downloads/HeightMap_Datasets/horizontal_stairs.hdf5", "r")

    height_maps = load_height_maps(data, 5000)

    print("Total Height Maps: ", len(height_maps))

    for index, height_map in enumerate(height_maps):
        print("Plotting Height Map: ", index, height_map.shape) 

        # show_height_map(height_map, delay=0)

        if np.mean(height_map) > 0.1:
            plot_and_compute_stats(height_map, debug=True, display=True)