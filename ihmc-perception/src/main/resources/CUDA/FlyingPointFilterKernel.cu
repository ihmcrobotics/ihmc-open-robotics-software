extern "C"
/**
 * Computes the median value from a given window of pixel values.
 *
 * @param window Pointer to an array containing pixel values.
 * @param size Number of elements in the window.
 * @return The median value.
 */
__device__ float computeMedian(float* window, int size)
{
    // Sort the window using Bubble Sort (can be replaced with a faster sort)
    for (int i = 0; i < size - 1; ++i)
    {
        for (int j = 0; j < size - i - 1; ++j)
        {
            if (window[j] > window[j + 1])
            {
                float temp = window[j];
                window[j] = window[j + 1];
                window[j + 1] = temp;
            }
        }
    }

    // Return median based on even or odd size
    if (size % 2 == 0)
    {
        return (window[size / 2 - 1] + window[size / 2]) / 2.0f;
    }
    else
    {
        return window[size / 2];
    }
}

extern "C"
/**
 * CUDA kernel to remove flying points from a depth map.
 *
 * @param in Input depth map (GPU memory).
 * @param pitchIn Pitch (stride) of input memory layout.
 * @param out Output depth map (GPU memory).
 * @param pitchOut Pitch (stride) of output memory layout.
 * @param rows Number of rows in the depth map.
 * @param cols Number of columns in the depth map.
 */
__global__ void filterFlyingPoints(unsigned short *in, size_t pitchIn,
                                   unsigned short *out, size_t pitchOut,
                                   int rows, int cols)
{
    // Compute the pixel coordinates for this thread
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    // Ensure thread operates within valid image boundaries
    if (xIndex >= cols || yIndex >= rows) return;

    // Read the depth value of the current pixel
    unsigned short *inputPixel = (unsigned short *)((char *)in + yIndex * pitchIn) + xIndex;
    unsigned short depthValue = *inputPixel;

    // Define a 3x3 neighborhood window
    const int windowSize = 3;
    const int halfWindow = windowSize / 2;
    float window[windowSize * windowSize];
    int count = 0;

    // Collect neighboring pixel values into the window
    for (int dx = -halfWindow; dx <= halfWindow; ++dx)
    {
        for (int dy = -halfWindow; dy <= halfWindow; ++dy)
        {
            int nx = xIndex + dx;
            int ny = yIndex + dy;

            if (nx >= 0 && nx < cols && ny >= 0 && ny < rows)
            {
                unsigned short *neighborPixel = (unsigned short *)((char *)in + ny * pitchIn) + nx;
                window[count++] = *neighborPixel;
            }
        }
    }

    // Compute the median value for filtering
    float median = computeMedian(window, count);

    // Write the filtered value to the output depth map
    unsigned short *outputPixel = (unsigned short *)((char *)out + yIndex * pitchOut) + xIndex;

    // If the depth value deviates too much from the median, mark it as a flying point
    if (fabsf(depthValue - median) >= 0.2f * median)
    {
        *outputPixel = static_cast<unsigned short>(median); // Set to median to remove flying point
    }
    else
    {
        *outputPixel = static_cast<unsigned short>(depthValue); // Keep original depth value
    }
}






