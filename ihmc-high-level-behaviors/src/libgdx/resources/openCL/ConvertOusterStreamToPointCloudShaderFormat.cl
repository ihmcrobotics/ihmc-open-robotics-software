// ouster in is
// fusedOut is 10 floats: X,Y,Z,R,G,B,A,0.01,1.0,0.0
__kernel void convertOusterSteamToPointCloudShaderFormat(__global unsigned char* ousterIn, __global float* fusedOut)
{
   int gid = get_global_id(0); // index of point

   int ousterInIndex = gid * 48;
   float* ousterFloats = &ousterIn[ousterInIndex];
   float ousterX = ousterFloats[0];
   float ousterY = ousterFloats[1];
   float ousterZ = ousterFloats[2];
   float r = 0.0f;
   float g = 0.0f;
   float b = 0.0f;
   float a = 1.0f;
   float size = 0.01f; // should be a parameter

   int fusedOutIndex = gid * 10;
   fusedOut[fusedOutIndex + 0] = ousterX;
   fusedOut[fusedOutIndex + 1] = ousterY;
   fusedOut[fusedOutIndex + 2] = ousterZ;
   fusedOut[fusedOutIndex + 3] = r;
   fusedOut[fusedOutIndex + 4] = g;
   fusedOut[fusedOutIndex + 5] = b;
   fusedOut[fusedOutIndex + 6] = a;
   fusedOut[fusedOutIndex + 7] = size;
   fusedOut[fusedOutIndex + 8] = 1.0f;
   fusedOut[fusedOutIndex + 9] = 0.0f;
}