__kernel void vectorAddition(__global float* a)
{
   int gid = get_global_id(0);
   a[gid] += a[gid];
}