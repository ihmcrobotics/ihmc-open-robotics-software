import numpy as np
import cv2

def generate_height_map():

    height_maps = dict()

    # create a flat terrain
    flat_height_map = np.zeros((200, 200), dtype=np.float32)
    height_maps["flat"] = flat_height_map

    # create stairs
    stairs_height_map = create_stairs(center_x=75, center_y=25, step_height=0.2, step_width=25, step_length=50, number_of_steps=6)
    height_maps["stairs"] = stairs_height_map

    # create a at the exact same place as the stairs to convert it to a ramp
    ramp_height_map = ramp_rectangle(row_start=75, row_end=125, col_start=25, col_end=175, offset=0.0, gradient=0.01, direction=1)
    height_maps["ramp"] = ramp_height_map
    
    # create a full ramp of 200 x 200
    full_ramp_height_map = ramp_rectangle(row_start=0, row_end=200, col_start=0, col_end=200, offset=0.0, gradient=0.001, direction=1)
    height_maps["full_ramp"] = full_ramp_height_map

    # create undulating terrain with sinusoidal function in just one direction
    undulating_height_map_1d = create_undulating_terrain_1d(amplitude=0.6, frequency=0.1)
    height_maps["undulating_1d"] = undulating_height_map_1d
    
    # create undulating terrain with sinusoidal function in both directions
    undulating_height_map_2d = create_undulating_terrain_2d(amplitude_x=0.6, frequency_x=0.1, amplitude_y=0.6, frequency_y=0.1)
    height_maps["undulating_2d"] = undulating_height_map_2d


    # create a random block field
    random_block_field_height_map = create_random_block_field(number_of_blocks=10, block_size=10, block_height=0.2)
    height_maps["random_block_field"] = random_block_field_height_map

    return height_maps


def create_undulating_terrain_1d(amplitude, frequency):
    height_map = np.zeros((200, 200), dtype=np.float32)
    for i in range(height_map.shape[0]):
        for j in range(height_map.shape[1]):
            height_map[i, j] = amplitude * np.sin(frequency * i)
    return height_map

def create_undulating_terrain_2d(amplitude_x, frequency_x, amplitude_y, frequency_y):
    height_map = np.zeros((200, 200), dtype=np.float32)
    for i in range(height_map.shape[0]):
        for j in range(height_map.shape[1]):
            height_map[i, j] = amplitude_x * np.sin(frequency_x * i) + amplitude_y * np.sin(frequency_y * j)
    return height_map

def create_stairs(center_x, center_y, step_height, step_width, step_length, number_of_steps):
    height_map = np.zeros((200, 200), dtype=np.float32)
    # use elevate_rectangle to create a height map that represents a top-down view of stairs with four step ups
    for i in range(number_of_steps):
        height_map = elevate_rectangle(center_x, center_x + step_length, center_y, center_y + step_width, 0.1 + step_height * i)
        center_y += step_width

    return height_map
    

def elevate_rectangle(row_start, row_end, col_start, col_end, offset):
    height_map = np.zeros((200, 200), dtype=np.float32)
    height_map[row_start:row_end, col_start:col_end] = offset
    return height_map

def ramp_rectangle(row_start, row_end, col_start, col_end, offset, gradient, direction):
    height_map = np.zeros((200, 200), dtype=np.float32)
    for i in range(height_map.shape[0]):
        for j in range(height_map.shape[1]):
            if i > row_start and i < row_end and j > col_start and j < col_end:
                height_map[i, j] = offset + gradient * direction * (j - row_start)


    return height_map

# use elevate_rectangle to generate N random blocks of size 10 x 10 with specified height
def create_random_block_field(number_of_blocks, block_size, block_height):
    height_map = np.zeros((200, 200), dtype=np.float32)
    for i in range(number_of_blocks):
        row_start = np.random.randint(0, 200)
        col_start = np.random.randint(0, 200)
        height_map[row_start:row_start + block_size, col_start:col_start + block_size] = block_height
    return height_map

def compute_terrain_cost_map(height_map):
    # convert height map to float32
    height_map = height_map.astype(np.float32)

    # perform bilateral filtering to smooth the height map
    height_map = cv2.bilateralFilter(height_map, 5, 75, 75)

    # for each cell compute the dot product of the normal vector with the Z axis. Use sobel filter to compute the normal vector
    # use sobel filter to compute the normal vector
    sobel_x = cv2.Sobel(height_map, cv2.CV_32F, 1, 0, ksize=3)
    sobel_y = cv2.Sobel(height_map, cv2.CV_32F, 0, 1, ksize=3)

    # compute the normal vector
    normal_vector = np.zeros((height_map.shape[0], height_map.shape[1], 3), dtype=np.float32)
    normal_vector[:, :, 0] = -sobel_x
    normal_vector[:, :, 1] = -sobel_y
    normal_vector[:, :, 2] = 1

    # normalize the normal vector
    normal_vector = normal_vector / np.linalg.norm(normal_vector, axis=2, keepdims=True)

    # compute the cost map
    cost_map = np.abs(normal_vector[:, :, 2])

    # threshold the cost map by setting everything below 0.9 to 0
    cost_map[cost_map < 0.999] = 0

    # set the values to be between 0 and 255
    cost_map = cost_map / np.max(cost_map) * 255

    # enlarge image for plotting
    # cost_map = cv2.resize(cost_map, (800, 800), interpolation=cv2.INTER_NEAREST)

    return cost_map

def compute_contact_map(terrain_cost_map):
    # the possible distance metrics include cv2.DIST_L1, cv2.DIST_L2, cv2.DIST_C, 
    # cv2.DIST_L12, cv2.DIST_FAIR, cv2.DIST_WELSCH, and cv2.DIST_HUBER
    contact_map = np.zeros((terrain_cost_map.shape[0], terrain_cost_map.shape[1]), dtype=np.float32)
    contact_map = cv2.distanceTransform(terrain_cost_map.astype(np.uint8), cv2.DIST_L2, 5)  
    contact_map = contact_map / np.max(contact_map) * 255
    return contact_map
    
def compute_pattern_stats(height_map):
    # create height map for plotting between 0 and 255
    height_map_for_plotting = height_map - np.min(height_map)
    height_map_for_plotting = height_map_for_plotting / np.max(height_map_for_plotting) * 255
    
    terrain_cost = compute_terrain_cost_map(height_map)
    contact_map = compute_contact_map(terrain_cost)

    plot_terrain_maps(height_map_for_plotting, terrain_cost, contact_map)


def plot_terrain_maps(height_map, terrain_cost, contact_map):

    # create colored image for plotting
    height_map_image = np.stack([height_map, height_map, height_map], axis=2).astype(np.uint8)
    
    # brighten the height map for visualization
    # height_map_image = cv2.convertScaleAbs(height_map_image, alpha=255, beta=0)

    # convert to opencv colored image
    contact_map = np.stack([contact_map, contact_map, contact_map], axis=2).astype(np.uint8)

    # Visualize the contact map with green color
    contact_map[:, :, 1] = contact_map[:, :, 0]
    contact_map[:, :, 0] = 0
    contact_map[:, :, 2] = 0

    # convert terrain_cost to color image and stack both images side by side
    terrain_cost = np.stack([terrain_cost, terrain_cost, terrain_cost], axis=2).astype(np.uint8)

    # stack but preserve width of each image
    stacked_image = np.hstack((height_map_image, terrain_cost, contact_map))

    # enlarge image for plotting
    stacked_image = cv2.resize(stacked_image, (2400, 800), interpolation=cv2.INTER_NEAREST)


    cv2.imshow("Contact Map", stacked_image)


    