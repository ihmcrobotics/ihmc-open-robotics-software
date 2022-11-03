
kernel void unpackPointCloud(global float* parameters,
                             global int* decompressedPointBuffer,
                             global float* discretizedPointBuffer)
{
   int n = get_global_id(0);

   int latestSegmentIndex = parameters[0];
   float pointSize = parameters[1];
   int pointsPerSegment = parameters[2];
   float discretization = 0.003; // TODO: Make parameter

   int inputIntsPerPoint = 4;
   int inputStartIndex = n * inputIntsPerPoint;

   int floatsPerVertex = 8;         // TODO: Need to remove these constants
   int currentSegmentStart = latestSegmentIndex * pointsPerSegment * floatsPerVertex;
   int pointCloudStartIndex = currentSegmentStart + n * floatsPerVertex;

   discretizedPointBuffer[pointCloudStartIndex] = decompressedPointBuffer[inputStartIndex] * discretization;
   discretizedPointBuffer[pointCloudStartIndex + 1] = decompressedPointBuffer[inputStartIndex + 1] * discretization;
   discretizedPointBuffer[pointCloudStartIndex + 2] = decompressedPointBuffer[inputStartIndex + 2] * discretization;

   int colorRGBA8888 = decompressedPointBuffer[inputStartIndex + 3];

   int rInt = (colorRGBA8888 >> 24) & 0xFF;
   int gInt = (colorRGBA8888 >> 16) & 0xFF;
   int bInt = (colorRGBA8888 >> 8) & 0xFF;
   int aInt = colorRGBA8888 & 0xFF;

   float r = rInt / 255.0;
   float g = gInt / 255.0;
   float b = bInt / 255.0;
   float a = aInt / 255.0;

   discretizedPointBuffer[pointCloudStartIndex + 3] = r;
   discretizedPointBuffer[pointCloudStartIndex + 4] = g;
   discretizedPointBuffer[pointCloudStartIndex + 5] = b;
   discretizedPointBuffer[pointCloudStartIndex + 6] = a;

   discretizedPointBuffer[pointCloudStartIndex + 7] = pointSize;    // TODO: Need to get rid of this by using uniform
}