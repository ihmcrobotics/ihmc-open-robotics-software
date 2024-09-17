import cupy as cp

# Create a 10x10 matrix filled with 1s on the GPU
matrix = cp.ones((10, 10), dtype=cp.float32)

# Perform an operation, e.g., adding 1 to each element
matrix += 1

# Convert the result back to NumPy array for CPU operations if needed
result = cp.asnumpy(matrix)

print(result)