/*
 * This is an example CUDA kernel.
 * Performs element wise addition of two uint16 matrices.
 * As in, result[i] = matrixA[i] + matrixB[i].
 * All passed in matrices must be of the same size, and hold uint16 data.
 * This kernel is designed to be launched in 2 dimensions (X, Y).
 *
 * This kernel is designed to work with matrices allocated using cudaMallocPitch().
 * (see: https://docs.nvidia.com/cuda/cuda-runtime-api/group__CUDART__MEMORY.html#group__CUDART__MEMORY_1g32bd7a39135594788a542ae72217775c)
 * It so happens that OpenCV's GpuMats use cudaMallocPitch() for memory allocation,
 * so this kernel is compatible with GpuMats. Simply pass in gpuMat.data() as the matrix.
 *
 * **********************************************************************************
 * * EXTRA INFO ABOUT cudaMallocPitch(), AND ACCESSING ELEMENTS WITHIN THE MATRICES *
 * **********************************************************************************
 * cudaMallocPitch() is used to allocate 2D matrices of a given width and height.
 * However, it may (and most often does) allocate extra memory at the end of each row
 * to meet certain alignment requirements.
 * The width of each row plus the padding size is known as "pitch" or "step."
 *
 * To put it into picture, assume we want to allocate a 3 * 3 matrix.
 * cudaMallocPitch() may allocate memory as such, where X marks padding:
 *
 * |----pitch=4----|
 * |--width=3--|
 * +---+---+---+---+
 * |   |   |   | X |
 * +---+---+---+---+
 * |   |   |   | X |
 * +---+---+---+---+
 * |   |   |   | X |
 * +---+---+---+---+
 *
 * Although we think of the matrix as being 2D, in memory it is stored as a 1D array:
 *
 * |----pitch=4----|
 * |--width=3--|
 * [   |   |   | X |   |   |   | X |   |   |   | X ]
 *
 * To find the memory for an element at a specific row and column, we use the following formulas:
 *  - T* rowAddress = (T*) ((char*) matrix + row * pitch);
 *  - T* elementAddress = rowAddress + column;
 *
 * Or, putting it together (as seen in cudaMallocPitch() documentation):
 *  - T* elementAddress = (T*) ((char*) matrix + row * pitch) + column;
 */
extern "C"
__global__
void element_wise_add(unsigned short * matrixA, size_t pitchA,
                      unsigned short * matrixB, size_t pitchB,
                      unsigned short * result, size_t pitchResult,
                      int rows, int cols)
{
    // Find the X index and stride of this thread
    int indexX = blockIdx.x * blockDim.x + threadIdx.x;
    int strideX = blockDim.x * gridDim.x;

    // Find the Y index and stride of this thread as well (we're working in 2D)
    int indexY = blockIdx.y * blockDim.y + threadIdx.y;
    int strideY = blockDim.y * gridDim.y;

    // Use a grid-stride loop
    // see: https://developer.nvidia.com/blog/cuda-pro-tip-write-flexible-kernels-grid-stride-loops/
    // also: https://stackoverflow.com/a/22607308
    for (int y = indexY; y < rows; y += strideY)
    {
        // We're looking at row y, so we find the beginning address of row y
        // using the first formula: T* rowAddress = (T*) ((char*) matrix + row * pitch);
        unsigned short * matrixARow = (unsigned short*)((char*) matrixA + y * pitchA);
        unsigned short * matrixBRow = (unsigned short*)((char*) matrixB + y * pitchB);
        unsigned short * resultRow = (unsigned short*)((char*) result + y * pitchResult);

        for (int x = indexX; x < cols; x += strideX)
        {
            // We're accessing column x within each row.
            // Same as doing: T* elementAddress = rowAddress + column; (the second formula)
            resultRow[x] = matrixARow[x] + matrixBRow[x];
        }
    }
}
