namespace Utils
{
    // Returns the X coordinate of this thread within the entire grid of threads.
    __device__ int getThreadCoordX()
    {
        return blockIdx.x * blockDim.x + threadIdx.x;
    }

    // Returns the Y coordinate of this thread within the entire grid of threads.
    __device__ int getThreadCoordY()
    {
        return blockIdx.y * blockDim.y + threadIdx.y;
    }

    // Returns the Z coordinate of this thread within the entire grid of threads.
    __device__ int getThreadCoordZ()
    {
        return blockIdx.z * blockDim.z + threadIdx.z;
    }

    // Returns the stride in the X dimension needed for a grid-stride loop
    __device__ int getStrideX()
    {
        return blockDim.x * gridDim.x;
    }

    // Returns the stride in the Y dimension needed for a grid-stride loop
    __device__ int getStrideY()
    {
        return blockDim.y * gridDim.y;
    }

    // Returns the stride in the Z dimension needed for a grid-stride loop
    __device__ int getStrideZ()
    {
        return blockDim.z * gridDim.z;
    }
}