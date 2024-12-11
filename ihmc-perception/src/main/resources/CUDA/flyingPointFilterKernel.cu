extern "C"
#define TILE_SIZE 16

__device__ float computeMedian(float* window, int size) {
    // Sort the window (bubble sort for simplicity; replace with better sorting if needed)
    for (int i = 0; i < size - 1; ++i) {
        for (int j = 0; j < size - i - 1; ++j) {
            if (window[j] > window[j + 1]) {
                float temp = window[j];
                window[j] = window[j + 1];
                window[j + 1] = temp;
            }
        }
    }
    // Return the median value
    return window[size / 2];
}

 extern "C" __global__ void filterFlyingPoints(unsigned short *in, size_t pitchIn, unsigned short *out, size_t pitchOut, int rows, int cols)
 {
    // Get the thread's pixel coordinates
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= cols || y >= rows) return;

    // Pointer to this thread's pixel in the depth map
    unsigned short *inRow = (unsigned short *)((char *)in + (y * pitchIn));
    unsigned short depthValue = *(inRow + x);

    unsigned short *outRow = (unsigned short *)((char *)out + (y * pitchOut));

    // Neighborhood window for smoothing (e.g., 3x3)
    const int windowSize = 3;
    const int halfWindow = windowSize / 2;
    float window[windowSize * windowSize];
    int count = 0;

    // Collect neighbor values
    for (int dx = -halfWindow; dx <= halfWindow; ++dx) {
        for (int dy = -halfWindow; dy <= halfWindow; ++dy) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < cols && ny >= 0 && ny < rows) {
               unsigned short* neighborRow = (unsigned short*)((char*)in + (ny * pitchIn));
               unsigned short neighborPixelValue = *(neighborRow + nx);
               window[count++] = neighborPixelValue;
            }
        }
    }

    // Compute the median value for smoothing
    float median = computeMedian(window, count);

    // Replace current pixel value with the median
     *(outRow + x) = static_cast<unsigned short>(median);

    // Statistical analysis: compute local mean and standard deviation
    float sum = 0.0f, sumSquared = 0.0f;
    for (int i = 0; i < count; ++i) {
        sum += window[i];
        sumSquared += window[i] * window[i];
    }
    float mean = sum / count;
    float variance = (sumSquared / count) - (mean * mean);
    float stdDev = sqrtf(variance);

    // Invalidate pixel if it deviates too much from the local mean
    if (fabsf(depthValue - mean) > 1.5 * stdDev) {
        *(outRow + y)= static_cast<unsigned short>(0.0f);
    }
}