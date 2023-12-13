#define ZED_FOCAL_LENGTH_X 0
#define ZED_FOCAL_LENGTH_Y 1
#define ZED_PRINCIPAL_POINT_X 2
#define ZED_PRINCIPAL_POINT_Y 3
#define ZED_DEPTH_DISCRETIZATION 4
#define ZED_IMAGE_WIDTH 5
#define ZED_IMAGE_HEIGHT 6
#define REALSENSE_FOCAL_LENGTH_X 7
#define REALSENSE_FOCAL_LENGTH_Y 8
#define REALSENSE_PRINCIPAL_POINT_X 9
#define REALSENSE_PRINCIPAL_POINT_Y 10
#define REALSENSE_DEPTH_DISCRETIZAION 11
#define REALSENSE_IMAGE_WIDTH 12
#define REALSENSE_IMAGE_HEIGHT 13

kernel void removeDepthOverlap(read_only image2d_t zedInput,
                               read_only image2d_t realsenseInput,
                               write_only image2d_t outputImage,
                               global float* parameters,
                               global float* zedToRealsenseTransform)
{
   int x = get_global_id(0);
   int y = get_global_id(1);

   uint zedDepth = read_imageui(zedInput, (int2) (x, y)).x;
   float zedDepthInMeters = zedDepth * parameters[ZED_DEPTH_DISCRETIZATION];
   float3 zedDepthFramePoint = (float3) (zedDepthInMeters,
                                        -(x - parameters[ZED_PRINCIPAL_POINT_X]) / parameters[ZED_FOCAL_LENGTH_X] * zedDepthInMeters,
                                        -(y - parameters[ZED_PRINCIPAL_POINT_Y]) / parameters[ZED_FOCAL_LENGTH_Y] * zedDepthInMeters);

   float3 realsenseDepthFramePoint = transformPoint3D32(zedDepthFramePoint, zedToRealsenseTransform);

   float yaw = -angle(1.0f, 0.0f, realsenseDepthFramePoint.x, realsenseDepthFramePoint.y);
   int realsensePixelColumn = round(parameters[REALSENSE_PRINCIPAL_POINT_X] + parameters[REALSENSE_FOCAL_LENGTH_X] * tan(yaw));

   float pitch = -angle(1.0f, 0.0f, realsenseDepthFramePoint.x, realsenseDepthFramePoint.z);
   int realsensePixelRow = round(parameters[REALSENSE_PRINCIPAL_POINT_Y] + parameters[REALSENSE_FOCAL_LENGTH_Y] * tan(pitch));

   bool pixelInBounds = intervalContains(realsensePixelColumn, 0, parameters[REALSENSE_IMAGE_WIDTH])
                        && intervalContains(realsensePixelRow, 0, parameters[REALSENSE_IMAGE_HEIGHT]);

   if (pixelInBounds)
   {
      zedDepth = 0;
   }
   
   write_imageui(outputImage, (int2) (x, y), zedDepth);
}