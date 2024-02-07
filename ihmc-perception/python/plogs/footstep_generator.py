from footstep_dataset_loader import visualize_plan
from hdf5_reader import *

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
    data = h5py.File("/home/bmishra/.ihmc/logs/perception/flat_ground.hdf5", "r")

    height_map = load_raw_height_maps(data, "/matrix", 0)

    print(height_map)
