#include "test_values.cuh"

extern "C"
__global__
void add(int * sum)
{
   *sum = a + b;
}

extern "C"
__global__
void subtract(int * sum)
{
   *sum = b - a;
}