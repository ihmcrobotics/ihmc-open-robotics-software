// ouster in is 48 bytes, float32s X, Y, Z, then throwaway stuff
// zed2 in is bytes B, G, R
// TODO: We'll need a camera transform to set those colors to the right ouster points
// fusedOut is 10 floats: X,Y,Z,R,G,B,A,0.01,1.0,0.0
__kernel void projectZED2ToOusterPoints(__global unsigned char* ousterIn, __global float* zed2In, __global float* fusedOut)
{
   int gid = get_global_id(0); // index of point

   int ousterInIndex = gid * 48;
   float* ousterFloats = &ousterIn[ousterInIndex];
   float ousterX = ousterFloats[0]
   float ousterY = ousterFloats[1]
   float ousterZ = ousterFloats[2]

   if (gid == 0)
   {
        printf("x: %f, y: %y, z: %z", ousterX, ousterY, ousterZ);
   }

   int fusedOutIndex = gid * 10;
   fusedOut[fusedOutIndex + 0] = ousterX;
   fusedOut[fusedOutIndex + 1] = ousterY;
   fusedOut[fusedOutIndex + 2] = ousterZ;
   fusedOut[fusedOutIndex + 3] = 1.0f;
   fusedOut[fusedOutIndex + 4] = 1.0f;
   fusedOut[fusedOutIndex + 5] = 1.0f;
   fusedOut[fusedOutIndex + 6] = 1.0f;
   fusedOut[fusedOutIndex + 7] = 0.01f;
   fusedOut[fusedOutIndex + 8] = 1.0f;
   fusedOut[fusedOutIndex + 9] = 0.0f;
}