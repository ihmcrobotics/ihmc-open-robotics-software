kernel void unpackPointCloud(global float* parameters, global int* decompressedPointBuffer, global float* pointCloudVertexBuffer)
{
   int n = get_global_id(0);

   int latestSegmentIndex = parameters[0];
   float pointSize = parameters[1];
   int pointsPerSegment = parameters[2];
   float discretization = parameters[3];

   int inputIntsPerPoint = 4;
   int inputStartIndex = n * inputIntsPerPoint;

   int floatsPerVertex = 8;
   int currentSegmentStart = latestSegmentIndex * pointsPerSegment * floatsPerVertex;
   int pointCloudStartIndex = currentSegmentStart + n * floatsPerVertex;

   pointCloudVertexBuffer[pointCloudStartIndex] = decompressedPointBuffer[inputStartIndex] * discretization;
   pointCloudVertexBuffer[pointCloudStartIndex + 1] = decompressedPointBuffer[inputStartIndex + 1] * discretization;
   pointCloudVertexBuffer[pointCloudStartIndex + 2] = decompressedPointBuffer[inputStartIndex + 2] * discretization;

   int colorRGBA8888 = decompressedPointBuffer[inputStartIndex + 3];

   int rInt = (colorRGBA8888 >> 24) & 0xFF;
   int gInt = (colorRGBA8888 >> 16) & 0xFF;
   int bInt = (colorRGBA8888 >> 8) & 0xFF;
   int aInt = colorRGBA8888 & 0xFF;

   float r = rInt / 255.0;
   float g = gInt / 255.0;
   float b = bInt / 255.0;
   float a = aInt / 255.0;

   pointCloudVertexBuffer[pointCloudStartIndex + 3] = r;
   pointCloudVertexBuffer[pointCloudStartIndex + 4] = g;
   pointCloudVertexBuffer[pointCloudStartIndex + 5] = b;
   pointCloudVertexBuffer[pointCloudStartIndex + 6] = a;

   pointCloudVertexBuffer[pointCloudStartIndex + 7] = pointSize;
}