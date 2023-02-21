#define HORIZONTAL_FIELD_OF_VIEW 0
#define VERTICAL_FIELD_OF_VIEW 1
#define DEPTH_IMAGE_WIDTH 2
#define DEPTH_IMAGE_HEIGHT 3
#define POINT_SIZE 4
#define POINTS_TO_ADD_ABOVE_AND_BELOW_FOR_COLOR_DETAIL 5

#define FISHEYE_IMAGE_WIDTH 0
#define FISHEYE_IMAGE_HEIGHT 1
#define FISHEYE_IMAGE_FOCAL_LENGTH_PIXELS_X 2
#define FISHEYE_IMAGE_FOCAL_LENGTH_PIXELS_Y 3
#define FISHEYE_IMAGE_FOCAL_PRINCIPAL_POINT_PIXELS_X 4
#define FISHEYE_IMAGE_FOCAL_PRINCIPAL_POINT_PIXELS_Y 5

kernel void imageToPointCloud(global float* parameters,
                              global float* ousterToWorldTransform,
                              read_only image2d_t discretizedDepthImage,
                              int useFisheyeColorImage,
                              global float* fisheyeParameters,
                              read_only image2d_t fThetaFisheyeRGBA8Image,
                              global float* ousterToFisheyeTransform,
                              global float* pointCloudVertexBuffer)
{
   int x = get_global_id(0);
   int y = get_global_id(1);

   int pointsToAddAboveAndBelowForColorDetail = parameters[POINTS_TO_ADD_ABOVE_AND_BELOW_FOR_COLOR_DETAIL];
   int totalVerticalPointsForColorDetail = 1 + 2 * pointsToAddAboveAndBelowForColorDetail;
   int ousterY = y / totalVerticalPointsForColorDetail;

   float discreteResolution = 0.001f;
   float eyeDepthInMeters = read_imageui(discretizedDepthImage, (int2) (x, ousterY)).x * discreteResolution;

   int xFromCenter = -x - (parameters[DEPTH_IMAGE_WIDTH] / 2); // flip
   int yFromCenter = ousterY - (parameters[DEPTH_IMAGE_HEIGHT] / 2);

   float angleXFromCenter = xFromCenter / (float) parameters[DEPTH_IMAGE_WIDTH] * parameters[HORIZONTAL_FIELD_OF_VIEW];
   float angleYFromCenter = yFromCenter / (float) parameters[DEPTH_IMAGE_HEIGHT] * parameters[VERTICAL_FIELD_OF_VIEW];

   // Create additional rotation only transform
   float16 angledRotationMatrix = newRotationMatrix();
   angledRotationMatrix = setToPitchOrientation(angleYFromCenter, angledRotationMatrix);
   angledRotationMatrix = prependYawRotation(angleXFromCenter, angledRotationMatrix);

   float3 beamFramePoint = (float3) (eyeDepthInMeters, 0.0f, 0.0f);
   float3 origin = (float3) (0.0f, 0.0f, 0.0f);
   float3 rotationMatrixRow0 = (float3) (angledRotationMatrix.s0, angledRotationMatrix.s1, angledRotationMatrix.s2);
   float3 rotationMatrixRow1 = (float3) (angledRotationMatrix.s3, angledRotationMatrix.s4, angledRotationMatrix.s5);
   float3 rotationMatrixRow2 = (float3) (angledRotationMatrix.s6, angledRotationMatrix.s7, angledRotationMatrix.s8);

   float3 ousterFramePoint = transformPoint3D32_2(beamFramePoint, rotationMatrixRow0, rotationMatrixRow1, rotationMatrixRow2, origin);

   float pointSize = parameters[POINT_SIZE] * eyeDepthInMeters;
   int verticalColorDetailOffsetIndex = y % totalVerticalPointsForColorDetail - pointsToAddAboveAndBelowForColorDetail;
   float verticalPointOffsetLocalZ = verticalColorDetailOffsetIndex * pointSize / 2.0f;
   float3 colorDetailPointOffsetLocalFrame = (float3) (0.0, 0.0, verticalPointOffsetLocalZ);
   float3 colorDetailPointOffset = transformPoint3D32_2(colorDetailPointOffsetLocalFrame,
                                                        rotationMatrixRow0,
                                                        rotationMatrixRow1,
                                                        rotationMatrixRow2,
                                                        origin);
   ousterFramePoint += colorDetailPointOffset;

   float3 worldFramePoint = transformPoint3D32(ousterFramePoint, ousterToWorldTransform);

   int pointStartIndex = (parameters[DEPTH_IMAGE_WIDTH] * y + x) * 8;

   if (eyeDepthInMeters == 0.0f)
   {
      pointCloudVertexBuffer[pointStartIndex]     = nan((uint) 0);
      pointCloudVertexBuffer[pointStartIndex + 1] = nan((uint) 0);
      pointCloudVertexBuffer[pointStartIndex + 2] = nan((uint) 0);
   }
   else
   {
      pointCloudVertexBuffer[pointStartIndex]     = worldFramePoint.x;
      pointCloudVertexBuffer[pointStartIndex + 1] = worldFramePoint.y;
      pointCloudVertexBuffer[pointStartIndex + 2] = worldFramePoint.z;
   }

   float pointColorR;
   float pointColorG;
   float pointColorB;
   float pointColorA;
   if (useFisheyeColorImage)
   {
      float3 fisheyeFramePoint = transformPoint3D32(ousterFramePoint, ousterToFisheyeTransform);

      float angleOfIncidence = angle3D(1.0f, 0.0f, 0.0f, fisheyeFramePoint.x, fisheyeFramePoint.y, fisheyeFramePoint.z);
      if (fabs(angleOfIncidence) < radians(92.5f))
      {
         // Equidistant fisheye camera model:
         // r = f * theta
         // theta is the azimuthal angle, i.e. the angle swept on the image plane to the image point cooresponding to the fisheye frame point.
         // We use atan2(-z, -y) with negative z and y to convert from right handed Y left, Z up to image X right, Y down.
         // f is a 2D vector of focal length in pixels decomposed into X and Y so that we can tune each separately, which is useful.
         // To get the components or r, we use sin and cos of theta (i.e. the azimuthal angle).
         // r is a 2D vector of pixels from the principle point on the image to the point cooresponding to the fisheye frame point.
         // We offset r by the principle point offsets to find the pixel row and column.
         // https://www.fujifilm.com/us/en/business/optical-devices/machine-vision-lens/fe185-series
         // https://en.wikipedia.org/wiki/Fisheye_lens#Mapping_function
         // https://www.ihmc.us/wp-content/uploads/2023/02/equidistant_fisheye_model-1024x957.jpeg
         float azimuthalAngle = atan2(-fisheyeFramePoint.z, -fisheyeFramePoint.y);
         int fisheyeCol = fisheyeParameters[FISHEYE_IMAGE_FOCAL_PRINCIPAL_POINT_PIXELS_X]
                        + fisheyeParameters[FISHEYE_IMAGE_FOCAL_LENGTH_PIXELS_X] * angleOfIncidence * cos(azimuthalAngle);
         int fisheyeRow = fisheyeParameters[FISHEYE_IMAGE_FOCAL_PRINCIPAL_POINT_PIXELS_Y]
                        + fisheyeParameters[FISHEYE_IMAGE_FOCAL_LENGTH_PIXELS_Y] * angleOfIncidence * sin(azimuthalAngle);

         if (intervalContains(fisheyeCol, 0, fisheyeParameters[FISHEYE_IMAGE_WIDTH])
          && intervalContains(fisheyeRow, 0, fisheyeParameters[FISHEYE_IMAGE_HEIGHT])
         {
            uint4 fisheyeColor = read_imageui(fThetaFisheyeRGBA8Image, (int2) (fisheyeCol, fisheyeRow));
            pointColorR = (fisheyeColor.x / 255.0f);
            pointColorG = (fisheyeColor.y / 255.0f);
            pointColorB = (fisheyeColor.z / 255.0f);
            pointColorA = (fisheyeColor.w / 255.0f);
         }
      }
      else
      {
         float4 rgba8888Color = calculateInterpolatedGradientColorFloat4(worldFramePoint.z);
         pointColorR = rgba8888Color.x;
         pointColorG = rgba8888Color.y;
         pointColorB = rgba8888Color.z;
         pointColorA = rgba8888Color.w;
      }
   }
   else
   {
      float4 rgba8888Color = calculateInterpolatedGradientColorFloat4(worldFramePoint.z);
      pointColorR = (rgba8888Color.x);
      pointColorG = (rgba8888Color.y);
      pointColorB = (rgba8888Color.z);
      pointColorA = (rgba8888Color.w);
   }

   pointCloudVertexBuffer[pointStartIndex + 3] = pointColorR;
   pointCloudVertexBuffer[pointStartIndex + 4] = pointColorG;
   pointCloudVertexBuffer[pointStartIndex + 5] = pointColorB;
   pointCloudVertexBuffer[pointStartIndex + 6] = pointColorA;
   pointCloudVertexBuffer[pointStartIndex + 7] = pointSize;
}
