namespace Utils
{
    extern "C"
    __device__ int getThreadCoordX()
    {
        return blockIdx.x * blockDim.x + threadIdx.x;
    }

    extern "C"
    __device__ int getThreadCoordY()
    {
        return blockIdx.y * blockDim.y + threadIdx.y;
    }

    extern "C"
    __device__ int getThreadCoordZ()
    {
        return blockIdx.z * blockDim.z + threadIdx.z;
    }

    extern "C"
    __device__ int getStrideX()
    {
        return blockDim.x * gridDim.x;
    }

    extern "C"
    __device__ int getStrideY()
    {
        return blockDim.y * gridDim.y;
    }

    extern "C"
    __device__ int getStrideZ()
    {
        return blockDim.z * gridDim.z;
    }
}