#define SLAVE_FOCAL_LENGTH_X 0
#define SLAVE_FOCAL_LENGTH_Y 1
#define SLAVE_PRINCIPAL_POINT_X 2
#define SLAVE_PRINCIPAL_POINT_Y 3
#define SLAVE_DEPTH_DISCRETIZATION 4
#define SLAVE_IMAGE_WIDTH 5
#define SLAVE_IMAGE_HEIGHT 6
#define MASTER_FOCAL_LENGTH_X 7
#define MASTER_FOCAL_LENGTH_Y 8
#define MASTER_PRINCIPAL_POINT_X 9
#define MASTER_PRINCIPAL_POINT_Y 10
#define MASTER_DEPTH_DISCRETIZAION 11
#define MASTER_IMAGE_WIDTH 12
#define MASTER_IMAGE_HEIGHT 13

kernel void removeDepthOverlap(read_only image2d_t slaveImage,
                               write_only image2d_t outputImage,
                               global float* parameters,
                               global float* slaveToMasterTransform)
{
   int x = get_global_id(0);
   int y = get_global_id(1);

   uint slaveDepth = read_imageui(slaveImage, (int2) (x, y)).x;
   float slaveDepthInMeters = slaveDepth * parameters[SLAVE_DEPTH_DISCRETIZATION];
   float3 slaveFramePoint = 
               (float3) (slaveDepthInMeters,
                        -(x - parameters[SLAVE_PRINCIPAL_POINT_X]) / parameters[SLAVE_FOCAL_LENGTH_X] * slaveDepthInMeters,
                        -(y - parameters[SLAVE_PRINCIPAL_POINT_Y]) / parameters[SLAVE_FOCAL_LENGTH_Y] * slaveDepthInMeters);

   float3 masterFramePoint = transformPoint3D32(slaveFramePoint, slaveToMasterTransform);

   float yaw = -angle(1.0f, 0.0f, masterFramePoint.x, masterFramePoint.y);
   int masterImagePixelColumn = round(parameters[MASTER_PRINCIPAL_POINT_X] + parameters[MASTER_FOCAL_LENGTH_X] * tan(yaw));

   float pitch = -angle(1.0f, 0.0f, masterFramePoint.x, masterFramePoint.z);
   int masterImagePixelRow = round(parameters[MASTER_PRINCIPAL_POINT_Y] + parameters[MASTER_FOCAL_LENGTH_Y] * tan(pitch));

   bool isOverlapping = intervalContains(masterImagePixelColumn, 10, parameters[MASTER_IMAGE_WIDTH] - 10)
                        && intervalContains(masterImagePixelRow, 10, parameters[MASTER_IMAGE_HEIGHT] - 10);

   if (isOverlapping)
   {
      slaveDepth = 0;
   }
   
   write_imageui(outputImage, (int2) (x, y), slaveDepth);
}