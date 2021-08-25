__kernel void filterOusterXYZ(__global float* in, __global float* out)
{
   int gid = get_global_id(0);
   out[gid * 27] += in[gid * 12];
   out[gid * 27 + 1] += in[gid * 12 + 1];
   out[gid * 27 + 2] += in[gid * 12 + 2];
}