#define CUT_OUT_IMAGE_FOCAL_LENGTH_X 0
#define CUT_OUT_IMAGE_FOCAL_LENGTH_Y 1
#define CUT_OUT_IMAGE_PRINCIPAL_POINT_X 2
#define CUT_OUT_IMAGE_PRINCIPAL_POINT_Y 3
#define CUT_OUT_IMAGE_DEPTH_DISCRETIZATION 4
#define CUT_OUT_IMAGE_WIDTH 5
#define CUT_OUT_IMAGE_HEIGHT 6
#define BASE_IMAGE_FOCAL_LENGTH_X 7
#define BASE_IMAGE_FOCAL_LENGTH_Y 8
#define BASE_IMAGE_PRINCIPAL_POINT_X 9
#define BASE_IMAGE_PRINCIPAL_POINT_Y 10
#define BASE_IMAGE_DEPTH_DISCRETIZAION 11
#define BASE_IMAGE_WIDTH 12
#define BASE_IMAGE_HEIGHT 13
#define ALLOWED_OVERLAP 14

kernel void removeDepthOverlap(read_only image2d_t imageToCut,
                               write_only image2d_t outputImage,
                               global float* parameters,
                               global float* cutOutToBaseImageTransform)
{
   int x = get_global_id(0);
   int y = get_global_id(1);

   uint depth = read_imageui(imageToCut, (int2) (x, y)).x;
   float depthInMeters = depth * parameters[CUT_OUT_IMAGE_DEPTH_DISCRETIZATION];
   float3 cutOutImageFramePoint = 
               (float3) (depthInMeters,
                        -(x - parameters[CUT_OUT_IMAGE_PRINCIPAL_POINT_X]) / parameters[CUT_OUT_IMAGE_FOCAL_LENGTH_X] * depthInMeters,
                        -(y - parameters[CUT_OUT_IMAGE_PRINCIPAL_POINT_Y]) / parameters[CUT_OUT_IMAGE_FOCAL_LENGTH_Y] * depthInMeters);

   float3 baseImageFramePoint = transformPoint3D32(cutOutImageFramePoint, cutOutToBaseImageTransform);

   float yaw = -angle(1.0f, 0.0f, baseImageFramePoint.x, baseImageFramePoint.y);
   int baseImagePixelColumn = round(parameters[BASE_IMAGE_PRINCIPAL_POINT_X] + parameters[BASE_IMAGE_FOCAL_LENGTH_X] * tan(yaw));

   float pitch = -angle(1.0f, 0.0f, baseImageFramePoint.x, baseImageFramePoint.z);
   int baseImagePixelRow = round(parameters[BASE_IMAGE_PRINCIPAL_POINT_Y] + parameters[BASE_IMAGE_FOCAL_LENGTH_Y] * tan(pitch));

   bool isOverlapping = intervalContains(baseImagePixelColumn, parameters[ALLOWED_OVERLAP],
                                         parameters[BASE_IMAGE_WIDTH] - parameters[ALLOWED_OVERLAP])
                        && intervalContains(baseImagePixelRow, parameters[ALLOWED_OVERLAP],
                                            parameters[BASE_IMAGE_HEIGHT] - parameters[ALLOWED_OVERLAP]);

   if (isOverlapping)
   {
      depth = 0;
   }
   
   write_imageui(outputImage, (int2) (x, y), depth);
}