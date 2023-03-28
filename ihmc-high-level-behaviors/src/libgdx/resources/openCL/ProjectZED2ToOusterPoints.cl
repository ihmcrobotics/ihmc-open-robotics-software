// ouster in is 48 bytes, float32s X, Y, Z, then throwaway stuff
// zed2 in is bytes B, G, R
// TODO: We'll need a camera transform to set those colors to the right ouster points
// fusedOut is 10 floats: X,Y,Z,R,G,B,A,0.01,1.0,0.0
__kernel void projectZED2ToOusterPoints(__global unsigned char* ousterIn, __global unsigned char* zed2In, __global float* fusedOut)
{
   int gid = get_global_id(0); // index of point

   int ousterInIndex = gid * 48;
   // TODO: This does not compile. Convert the chars to float in a different way
   float* ousterFloats = &ousterIn[ousterInIndex];
   float ousterX = ousterFloats[0];
   float ousterY = ousterFloats[1];
   float ousterZ = ousterFloats[2];

   // TODO: transform 3D point to camera

   int zed2InIndex = gid * 3;
   float zed2B;
   float zed2G;
   float zed2R;
   if (gid >= 921600)
   {
      zed2B = 1.0f;
      zed2G = 1.0f;
      zed2R = 1.0f;
   }
   else
   {
      zed2B = convert_float(zed2In[zed2InIndex]) / 255.0f;
      zed2G = convert_float(zed2In[zed2InIndex + 1]) / 255.0f;
      zed2R = convert_float(zed2In[zed2InIndex + 2]) / 255.0f;
   }

   if (gid == 2000)
   {
      // printf("x: %f, y: %f, z: %f", ousterX, ousterY, ousterZ);
   }

   int fusedOutIndex = gid * 10;
   fusedOut[fusedOutIndex + 0] = ousterX;
   fusedOut[fusedOutIndex + 1] = ousterY;
   fusedOut[fusedOutIndex + 2] = ousterZ;
   fusedOut[fusedOutIndex + 3] = zed2R;
   fusedOut[fusedOutIndex + 4] = zed2G;
   fusedOut[fusedOutIndex + 5] = zed2B;
   fusedOut[fusedOutIndex + 6] = 1.0f;
   fusedOut[fusedOutIndex + 7] = 0.01f;
   fusedOut[fusedOutIndex + 8] = 1.0f;
   fusedOut[fusedOutIndex + 9] = 0.0f;
}