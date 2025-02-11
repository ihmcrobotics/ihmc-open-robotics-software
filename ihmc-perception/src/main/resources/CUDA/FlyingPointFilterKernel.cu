
// Computes the median value from a given window of pixel values.
// window = The size of the kernel to check its neighbors.

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

__device__ float computeMean(float* window, int size)
{
    float sum = 0.0f;
    for (int i = 0; i < size; ++i)
    {
        sum += window[i];
    }
    return sum / size;
}

__device__ float computeStdDev(float* window, int size, float mean)
{
    float sumSquares = 0.0f;
    for (int i = 0; i < size; ++i)
    {
        sumSquares += (window[i] - mean) * (window[i] - mean);
    }
    return sqrtf(sumSquares / size);
}

extern "C"
__global__ void filterFlyingPoints(unsigned short *in, size_t pitchIn,
                                   unsigned short *out, size_t pitchOut,
                                   int rows, int cols)
{
    int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    if (xIndex >= cols || yIndex >= rows) return;

    unsigned short *inputPixel = (unsigned short *)((char *)in + yIndex * pitchIn) + xIndex;
    unsigned short depthValue = *inputPixel;

    const int windowSize = 5;
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

    float median = computeMedian(window, count);
    float mean = computeMean(window, count);
    float stdDev = computeStdDev(window, count, mean);

    unsigned short *outputPixel = (unsigned short *)((char *)out + yIndex * pitchOut) + xIndex;

    // If the depth value deviates too much from the median, mark it as a flying point
    if (fabsf(depthValue - median) >= 0.02f * median || fabsf(depthValue - median) > 1.5f * stdDev)
    {
        *outputPixel = static_cast<unsigned short>(median); // Set to median to remove flying point
    }
    else
    {
        *outputPixel = static_cast<unsigned short>(depthValue); // Keep original depth value
    }
}