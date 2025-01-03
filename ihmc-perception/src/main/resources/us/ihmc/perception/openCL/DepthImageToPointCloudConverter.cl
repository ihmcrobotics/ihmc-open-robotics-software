#define DEPTH_IMAGE_WIDTH 0
#define FOCAL_LENGTH_X 1
#define FOCAL_LENGTH_Y 2
#define PRINCIPAL_POINT_X 3
#define PRINCIPAL_POINT_Y 4
#define DEPTH_DISCRETIZAION 5

kernel void convertDepthImageToPointCloud(read_only image2d_t depthImage,
                                          global float* parameters,
                                          global float* depthToWorldTransform,
                                          global float* pointCloudOutput)
{
   int x = get_global_id(0);
   int y = get_global_id(1);

   float depthInMeters = read_imageui(depthImage, (int2) (x, y)).x * parameters[DEPTH_DISCRETIZAION];
   float3 depthFramePoint = (float3) (depthInMeters, 
                                      -(x - parameters[PRINCIPAL_POINT_X]) / parameters[FOCAL_LENGTH_X] * depthInMeters, 
                                      -(y - parameters[PRINCIPAL_POINT_Y]) / parameters[FOCAL_LENGTH_Y] * depthInMeters);

   float3 worldFramePoint = transformPoint3D32(depthFramePoint, depthToWorldTransform);

   int pointStartIndex = (parameters[DEPTH_IMAGE_WIDTH] * y + x) * 4;
   pointCloudOutput[pointStartIndex + 0] = depthInMeters;
   pointCloudOutput[pointStartIndex + 1] = worldFramePoint.x;
   pointCloudOutput[pointStartIndex + 2] = worldFramePoint.y;
   pointCloudOutput[pointStartIndex + 3] = worldFramePoint.z;
}