// Computes the median value from a given window of pixel values.
// window = The size of the kernel to check its neighbors.

__device__ float computeMedian(float* window, int size)
{
    // Sort the window using Bubble Sort (can be replaced with a faster sort)
    // Bubble Sort works by repeatedly swapping adjacent elements if they are in the wrong order.
    // After each full pass, the largest unsorted element moves to its correct position.
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
        int mid = size / 2;
        float sum = window[mid - 1] + window[mid];
        return sum * 0.5f;
    }
    return window[size / 2];
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

__device__ float computePopulationStdDev(float* window, int size, float mean)
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

    // windowSize = 5 was chosen after visualizing 3x3, 5x5, and 7x7, as it gave the best result for the height map. It's a tuning parameter.
    const int windowSize = 5;

    const int halfWindow = windowSize / 2;
    float window[windowSize * windowSize];
    int count = 0;

    // Collect neighboring pixel values into the window
    for (int dx = -halfWindow; dx <= halfWindow; ++dx)
    {
        int nx = xIndex + dx;
        if (nx < 0 || nx >= cols)
            continue;

        for (int dy = -halfWindow; dy <= halfWindow; ++dy)
        {
            int ny = yIndex + dy;

            if (ny < 0 || ny >= rows)
                continue;

            // Skip the current cell
            if (nx == 0 && ny == 0)
                continue;

            unsigned short *neighborPixel = (unsigned short *)((char *)in + ny * pitchIn) + nx;
            window[count++] = *neighborPixel;
        }
    }

    float median = computeMedian(window, count);
    float mean = computeMean(window, count);
    float stdDev = computePopulationStdDev(window, count, mean);

    unsigned short *outputPixel = (unsigned short *)((char *)out + yIndex * pitchOut) + xIndex;

    // If the depth value deviates too much from the median, mark it as a flying point
    float diff = fabsf(depthValue - median);

    if (diff >= 0.02f * median || diff > 1.5f * stdDev)
    {
        *outputPixel = static_cast<unsigned short>(median); // Set to median to remove flying point
    }
    else
    {
        *outputPixel = static_cast<unsigned short>(depthValue); // Keep original depth value
    }
}