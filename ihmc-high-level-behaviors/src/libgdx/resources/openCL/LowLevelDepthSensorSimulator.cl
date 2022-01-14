kernel void lowLevelDepthSensorSimulator(read_only image2d_t normalizedDeviceCoordinateDepthImage,
                                         read_only image2d_t rgba8888ColorImage,
                                         write_only image2d_t metersDepthImage,
                                         write_only image2d_t pointCloudRenderingImage,
                                         global float* parameters)
{
   int x = get_global_id(0);
   int y = get_global_id(1);

   float cameraNear = parameters[0];
   float cameraFar = parameters[1];
   float principalOffsetXPixels = parameters[2];
   float principalOffsetYPixels = parameters[3];
   float focalLengthPixels = parameters[4];

   // From "How to render depth linearly in modern OpenGL with gl_FragCoord.z in fragment shader?"
   // https://stackoverflow.com/a/45710371/1070333
   float twoXCameraFarNear = 2.0f * cameraNear * cameraFar;
   float farPlusNear = cameraFar + cameraNear;
   float farMinusNear = cameraFar - cameraNear;
   float normalizedDeviceCoordinateZ = read_imagef(normalizedDeviceCoordinateDepthImage, (int2) (x,y)).x;
   float eyeDepth = (twoXCameraFarNear / (farPlusNear - normalizedDeviceCoordinateZ * farMinusNear));
   write_imagef(metersDepthImage, (int2) (x,y), eyeDepth);
}