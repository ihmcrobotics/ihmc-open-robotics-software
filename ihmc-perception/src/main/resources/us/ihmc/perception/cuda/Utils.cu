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

    template <typename T>
    __device__ void warpReduceAdd(volatile T* __restrict__ sharedArray, unsigned int threadIndex, unsigned int blockSize)
    {
        if (blockSize >= 64)                           sharedArray[threadIndex] += sharedArray[threadIndex + 32];
        if (blockSize >= 32) { if (threadIndex < 16) { sharedArray[threadIndex] += sharedArray[threadIndex + 16]; }}
        if (blockSize >= 16) { if (threadIndex <  8) { sharedArray[threadIndex] += sharedArray[threadIndex +  8]; }}
        if (blockSize >=  8) { if (threadIndex <  4) { sharedArray[threadIndex] += sharedArray[threadIndex +  4]; }}
        if (blockSize >=  4) { if (threadIndex <  2) { sharedArray[threadIndex] += sharedArray[threadIndex +  2]; }}
        if (blockSize >=  2) { if (threadIndex == 0) { sharedArray[0]           += sharedArray[1]; }}
    }

    /*
     * Sums all values provided in `value` by each thread to the `globalResult`.
     * `sharedArray` must be an array declared using `__shared__` with `sizeInBytes >= sizeof(T) * blockSize`
     * `globalResult` must be a pointer to global memory.
     * Additionally, the block size must be a power of 2.
     */
    template <typename T>
    __device__ void reduceAdd(T value, T* __restrict__ sharedArray, T* __restrict__ globalResult)
    {
        // Calculate the block size and this thread's index
        unsigned int blockSize = blockDim.x * blockDim.y * blockDim.z;
        unsigned int threadIndex = threadIdx.x
                                 + threadIdx.y * blockDim.x
                                 + threadIdx.z * blockDim.x * blockDim.y;

        // All threads load the value into the shared memory
        sharedArray[threadIndex] = value;
        __syncthreads();

        // Do the reduction in shared memory
        if (blockSize >= 1024) { if (threadIndex < 512) { sharedArray[threadIndex] += sharedArray[threadIndex + 512]; } __syncthreads(); }
        if (blockSize >=  512) { if (threadIndex < 256) { sharedArray[threadIndex] += sharedArray[threadIndex + 256]; } __syncthreads(); }
        if (blockSize >=  256) { if (threadIndex < 128) { sharedArray[threadIndex] += sharedArray[threadIndex + 128]; } __syncthreads(); }
        if (blockSize >=  128) { if (threadIndex <  64) { sharedArray[threadIndex] += sharedArray[threadIndex +  64]; } __syncthreads(); }

        // Once we have 64 elements (or less) left to reduce, we do it in a single warp and can avoid calling __syncthread()
        if (threadIndex < 32) warpReduceAdd(sharedArray, threadIndex, blockSize);

        // Finally all blocks sum their value to a single global result
        if (threadIndex == 0) atomicAdd(globalResult, sharedArray[0]);
    }
}
