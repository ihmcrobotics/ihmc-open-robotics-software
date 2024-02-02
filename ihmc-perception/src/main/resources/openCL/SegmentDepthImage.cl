#define DEPTH_DISCRETIZATION 0
#define DEPTH_PRINCIPAL_POINT_X 1
#define DEPTH_PRINCIPAL_POINT_Y 2
#define DEPTH_FOCAL_LENGTH_X 3
#define DEPTH_FOCAL_LENGTH_Y 4
#define MASK_PRINCIPAL_POINT_X 5
#define MASK_PRINCIPAL_POINT_Y 6
#define MASK_FOCAL_LENGTH_X 7
#define MASK_FOCAL_LENGTH_Y 8

kernel void segmentDepthImage(read_only image2d_t depthImage,
                              read_only image2d_t imageMask,
                              global float* parameters,
                              global float* depthToMaskTransform,
                              write_only image2d_t outputImage)
{
   int x = get_global_id(0);
   int y = get_global_id(1);

   float depthValue = read_imageui(depthImage, (int2) (x, y)).x;

   float depthInMeters = depthValue * parameters[DEPTH_DISCRETIZATION];
   float3 depthFramePoint = (float3) (depthInMeters, 
                                      -(x - parameters[DEPTH_PRINCIPAL_POINT_X]) / parameters[DEPTH_FOCAL_LENGTH_X] * depthInMeters, 
                                      -(y - parameters[DEPTH_PRINCIPAL_POINT_Y]) / parameters[DEPTH_FOCAL_LENGTH_Y] * depthInMeters);

   float3 maskFramePoint = transformPoint3D32(depthFramePoint, depthToMaskTransform);

   float yaw = -angle(1.0f, 0.0f, maskFramePoint.x, maskFramePoint.y);
   int maskImagePixelColumn = round(parameters[MASK_PRINCIPAL_POINT_X] + parameters[MASK_FOCAL_LENGTH_X] * tan(yaw));

   float pitch = -angle(1.0f, 0.0f, maskFramePoint.x, maskFramePoint.z);
   int maskImagePixelRow = round(parameters[MASK_PRINCIPAL_POINT_Y] + parameters[MASK_FOCAL_LENGTH_Y] * tan(pitch));

   float maskValue = read_imageui(imageMask, (int2) (maskImagePixelColumn, maskImagePixelRow)).x;
   if (maskValue > 0.0f)
      write_imageui(outputImage, (int2) (x, y), depthValue);
   else
      write_imageui(outputImage, (int2) (x, y), 0.0f);
}