package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.cuda.global.cudart;
import org.bytedeco.javacpp.FloatPointer;
import us.ihmc.log.LogTools;

import static org.bytedeco.cuda.global.cudart.*;

public class ExampleCUDAKernel
{

   private static final String KERNEL_TO_ADD_THE_VALUES_OF_TWO_ARRAYS = """
         extern "C"
         
         __global__
         void add_arrays(int n, float *x, float *y)
         {
            int index = blockIdx.x * blockDim.x + threadIdx.x;
            int stride = blockDim.x * gridDim.x;
            for (int i = index; i < n; i += stride)
               y[i] = x[i] + y[i];
         }
         """;

   // Even if you don't create a stream, CUDA will create a default one for you, so we make one ourselves to handle things better
   public ExampleCUDAKernel()
   {
      // Note that the name of this string doesn't have to match the name of the kernel, however for readability its ideal to have them match.
      String userFriendlyNameOfProgram = "userFriendlyNameOfProgram";
      // Note this name has to match the name of the CUDA kernel you want to run
      String kernelName = "add_arrays";

      // We create a stream to synchronize the method calls that happen on the GPU.
      CUstream_st stream = new CUstream_st();
      // Allocates the memory for the stream and makes sure the GPU knows about the stream
      cudart.cudaStreamCreate(stream);

      // This is going to hold the kernel source code
      CUDAProgram program = new CUDAProgram(userFriendlyNameOfProgram, KERNEL_TO_ADD_THE_VALUES_OF_TWO_ARRAYS);
      CUDAKernel kernel = program.loadKernel(kernelName);

      // Don't need this we can pass it in as a primitive type
      int arraySize = 5;
      FloatPointer cpuArrayX = new FloatPointer(1.0f, 2.0f, 3.0f, 4.0f, 5.0f);
      FloatPointer cpuArrayY = new FloatPointer(5.0f, 4.0f, 3.0f, 2.0f, 1.0f);

      FloatPointer gpuArrayX = new FloatPointer();
      FloatPointer gpuArrayY = new FloatPointer();

      //TODO look into this because it may mess up addition on the kernel
      cudaMallocAsync(gpuArrayX, (long) gpuArrayX.sizeof() * arraySize, stream);
      cudaMallocAsync(gpuArrayY, (long) gpuArrayY.sizeof() * arraySize, stream);

      cudaMemcpyAsync(gpuArrayX, cpuArrayX, (long) cpuArrayX.sizeof() * arraySize, cudaMemcpyDefault, stream);
      cudaMemcpyAsync(gpuArrayY, cpuArrayY, (long) cpuArrayY.sizeof() * arraySize, cudaMemcpyDefault, stream);

      kernel.withInt(arraySize).withPointer(gpuArrayX).withPointer(gpuArrayY);
      kernel.run(stream, new dim3(), new dim3(), 0);

      cudaMemcpyAsync(cpuArrayY, gpuArrayY, (long) gpuArrayY.sizeof() * arraySize, cudaMemcpyDefault, stream);

      cudaStreamSynchronize(stream);

      cudaFreeAsync(gpuArrayX, stream);
      cudaFreeAsync(gpuArrayY, stream);

      program.close();

      float[] test = new float[arraySize];
      LogTools.info(cpuArrayX.get(test).get(0));

      cpuArrayX.close();
      cpuArrayY.close();
      gpuArrayX.close();
      gpuArrayY.close();
      kernel.close();



      // At the end we have to destroy the stream to release the memory
      cudart.cudaStreamDestroy(stream);
   }

   public static void main(String[] args)
   {
      new ExampleCUDAKernel();
   }
}
