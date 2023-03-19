kernel void discretizePoints(global float* pointCloudBuffer, global int* discretizedIntBuffer, global float* parameters)
{
   int n = get_global_id(0);

   int floatsPerPoint = parameters[0];
   int intsPerPoint = parameters[1];
   float discreteResolution = parameters[2];
   int segmentIndex = parameters[3];
   int pointsPerSegment = parameters[4];

   float4 worldFramePoint;
   int worldPointStartIndex = segmentIndex * pointsPerSegment * floatsPerPoint + n * floatsPerPoint;
   worldFramePoint.x = pointCloudBuffer[worldPointStartIndex];
   worldFramePoint.y = pointCloudBuffer[worldPointStartIndex + 1];
   worldFramePoint.z = pointCloudBuffer[worldPointStartIndex + 2];

   int color = calculateInterpolatedGradientColorInt((double) worldFramePoint.z);

   int discreteX = (int) round(worldFramePoint.x / discreteResolution);
   int discreteY = (int) round(worldFramePoint.y / discreteResolution);
   int discreteZ = (int) round(worldFramePoint.z / discreteResolution);

   int pointStartIndex = n * intsPerPoint;
   discretizedIntBuffer[pointStartIndex]     = discreteX;
   discretizedIntBuffer[pointStartIndex + 1] = discreteY;
   discretizedIntBuffer[pointStartIndex + 2] = discreteZ;
   discretizedIntBuffer[pointStartIndex + 3] = color;
}