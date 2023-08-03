#define DEPTH_IMAGE_WIDTH 0
#define DEPTH_IMAGE_HEIGHT 1
#define LIDAR_ORIGIN_TO_BEAM_ORIGIN 2
#define DISCRETE_RESOLUTION 3
#define GRADIENT_MODE 4
#define USE_SINUSOIDAL_GRADIENT 5
#define POINT_SIZE 6
#define LEVEL_OF_COLOR_DETAIL 7
#define USE_FISHEYE_COLOR 8

#define FISHEYE_IMAGE_WIDTH 0
#define FISHEYE_IMAGE_HEIGHT 1
#define FISHEYE_IMAGE_FOCAL_LENGTH_PIXELS_X 2
#define FISHEYE_IMAGE_FOCAL_LENGTH_PIXELS_Y 3
#define FISHEYE_IMAGE_FOCAL_PRINCIPAL_POINT_PIXELS_X 4
#define FISHEYE_IMAGE_FOCAL_PRINCIPAL_POINT_PIXELS_Y 5

#define GRADIENT_MODE_WORLD_Z 0
#define GRADIENT_MODE_SENSOR_X 1

kernel void computeVertexBuffer(global float* parameters,
                                global float* altitudeAngles,
                                global float* azimuthAngles,
                                global float* ousterToWorldTransform,
                                read_only image2d_t discretizedDepthImage,
                                global float* fisheyeParameters,
                                read_only image2d_t fThetaFisheyeRGBA8Image,
                                global float* ousterToFisheyeTransform,
                                global float* pointCloudVertexBuffer)
{
   int x = get_global_id(0);
   int y = get_global_id(1);

   // Adds points above and below the lidar point symetrically
   // for the purpose of displaying more color details.
   int totalVerticalPointsForColorDetail = 1 + 2 * parameters[LEVEL_OF_COLOR_DETAIL];
   int ousterY = y / totalVerticalPointsForColorDetail;

   float eyeDepthInMeters = read_imageui(discretizedDepthImage, (int2) (x, ousterY)).x * parameters[DISCRETE_RESOLUTION];

   float encoderAngle = 2.0f * M_PI_F * (1.0f - ((float) x / (float) parameters[DEPTH_IMAGE_WIDTH]));
   float azimuthAngle = -azimuthAngles[ousterY];

   // Adjust the altitude angle to add points above and below for color detail
   int verticalColorDetailOffsetIndex = y % totalVerticalPointsForColorDetail - parameters[LEVEL_OF_COLOR_DETAIL];
   float altitudeStep = M_PI_F / 2.0 / (totalVerticalPointsForColorDetail * parameters[DEPTH_IMAGE_HEIGHT]);
   float altitudeAdjustment = verticalColorDetailOffsetIndex * altitudeStep;

   float altitudeAngle = altitudeAngles[ousterY] + altitudeAdjustment;

   // This uses the model from the user manual
   float beamLength = eyeDepthInMeters - parameters[LIDAR_ORIGIN_TO_BEAM_ORIGIN]; // Subtract the length of the sensor arm
   float3 ousterFramePoint = (float3)
      (-beamLength * cos(encoderAngle + azimuthAngle) * cos(altitudeAngle) + parameters[LIDAR_ORIGIN_TO_BEAM_ORIGIN] * cos(encoderAngle),
       -beamLength * sin(encoderAngle + azimuthAngle) * cos(altitudeAngle) + parameters[LIDAR_ORIGIN_TO_BEAM_ORIGIN] * sin(encoderAngle),
       beamLength * sin(altitudeAngle));

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

   float4 pointColor = (float4) (1.0f, 1.0f, 1.0f, 1.0f);
   bool appliedColorFromSensor = false;
   if (parameters[USE_FISHEYE_COLOR])
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
         int fisheyeCol = round(fisheyeParameters[FISHEYE_IMAGE_FOCAL_PRINCIPAL_POINT_PIXELS_X]
                              + fisheyeParameters[FISHEYE_IMAGE_FOCAL_LENGTH_PIXELS_X] * angleOfIncidence * cos(azimuthalAngle));
         int fisheyeRow = round(fisheyeParameters[FISHEYE_IMAGE_FOCAL_PRINCIPAL_POINT_PIXELS_Y]
                              + fisheyeParameters[FISHEYE_IMAGE_FOCAL_LENGTH_PIXELS_Y] * angleOfIncidence * sin(azimuthalAngle));

         if (intervalContains(fisheyeCol, 0, fisheyeParameters[FISHEYE_IMAGE_WIDTH])
          && intervalContains(fisheyeRow, 0, fisheyeParameters[FISHEYE_IMAGE_HEIGHT]))
         {
            uint4 fisheyeColor = read_imageui(fThetaFisheyeRGBA8Image, (int2) (fisheyeCol, fisheyeRow));
            pointColor = convert_float4(fisheyeColor) / 255.0f;
            appliedColorFromSensor = true;
         }
      }
   }
   if (!appliedColorFromSensor)
   {
      if (parameters[GRADIENT_MODE] == GRADIENT_MODE_WORLD_Z)
      {
         pointColor = calculateGradientColorOptionFloat4(worldFramePoint.z, parameters[USE_SINUSOIDAL_GRADIENT]);
      }
      else // GRADIENT_MODE_SENSOR_X
      {
         pointColor = calculateGradientColorOptionFloat4(eyeDepthInMeters, parameters[USE_SINUSOIDAL_GRADIENT]);
      }
   }

   pointCloudVertexBuffer[pointStartIndex + 3] = pointColor.x;
   pointCloudVertexBuffer[pointStartIndex + 4] = pointColor.y;
   pointCloudVertexBuffer[pointStartIndex + 5] = pointColor.z;
   pointCloudVertexBuffer[pointStartIndex + 6] = pointColor.w;
   pointCloudVertexBuffer[pointStartIndex + 7] = parameters[POINT_SIZE] * eyeDepthInMeters;
}
