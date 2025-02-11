extern "C"
__global__
void matrix_3d_example(unsigned short * matrixPointer, size_t pitchA, size_t layerSize, int rows, int cols, int layers)
{
    int indexX = blockIdx.x * blockDim.x + threadIdx.x;  // Column index
    int indexY = blockIdx.y * blockDim.y + threadIdx.y;  // Row index

    if (indexX >= cols || indexY >= rows)
        return; // Prevent out-of-bounds access

    // Loop over layers
    for (int layer = 0; layer < layers; layer++)
    {
        // Compute the base address of the current layer
        unsigned short * currentLayer = (unsigned short*)((char*) matrixPointer + layer * layerSize);

        // Compute row offset using pitchA
        unsigned short * matrixPointerRow = (unsigned short*)((char*) currentLayer + indexY * pitchA) + indexX;

        // Fetch value from the current layer
        int query_height_int = (int)(*matrixPointerRow);

        // Print out the value along with its position in the 3D matrix
        printf("Layer %d, Value at (%d, %d): %d\n", layer, indexX, indexY, query_height_int);
    }
}

